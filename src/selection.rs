use core::f32;
use std::f32::consts::FRAC_PI_2;

use glam::{Quat, Vec3};
use gluon_ipc::{Interface, Liveness, Node};
use rustc_hash::FxHashMap;
use stardust_xr_fusion::{
	Result,
	client::{Client, ClientHandler},
	drawable::{Lines, LinesExt},
	fields::{FieldRef, RayMarchResult},
	query::{InterfaceDependency, QueriedInterface, QueryableId},
	spatial::{
		BoundingBox, PartialTransform, Spatial, SpatialExt, SpatialInterface, SpatialRef, Transform,
	},
	spatial_query::{BeamQuery, BeamQueryHandle, BeamQueryHandlerHandler},
};
use stardust_xr_molecules::{
	lines::{LineExt, bounding_box},
	transformable::protocol::Poseable,
};
use tokio::sync::Mutex;
use tracing::warn;

/// max length one side of a bounding box may have before being filtered, in meters.
const MAX_BB_LENGTH: f32 = 1.0;
/// how many axes in a bounding box may be longer than the max before the object is being filtered.
const MAX_TOO_LONG_AXES: u8 = 1;

pub struct Selector {
	beams: Node<Beams>,
	query: BeamQueryHandle,
	spatial_interface: SpatialInterface,
	selection_lines: Lines,
	lines_spatial: Spatial,
	lines_spatial_ref: SpatialRef,
	target: Spatial,
	root: SpatialRef,
	selection: Option<Selection>,
}

impl Selector {
	pub async fn new(
		client: &Client<impl ClientHandler>,
		reference_space: SpatialRef,
		target: Spatial,
	) -> Result<Self> {
		let (lines_spatial, lines_spatial_ref) =
			Spatial::new(client, client.root(), Transform::IDENTITY).await?;
		let selection_lines = Lines::new(client, &lines_spatial, Vec::new()).await?;
		let (beams, beams_ref) = Beams {
			map: Default::default(),
			interface: client.spatial_interface().clone(),
			ref_space: reference_space.clone(),
		}
		.to_node()?;
		let query = client
			.spatial_query_interface()
			.beam_query(BeamQuery {
				handler: beams_ref.into_proxy(),
				interfaces: vec![InterfaceDependency {
					id: Poseable::ID.to_string(),
					optional: false,
				}],
				reference_spatial: reference_space,
				origin: [0.0; 3].into(),
				direction: Vec3::NEG_Z.into(),
				max_length: f32::MAX,
				margin: 0.01,
			})
			.await??;

		Ok(Selector {
			beams,
			query,
			spatial_interface: client.spatial_interface().clone(),
			selection_lines,
			lines_spatial,
			lines_spatial_ref,
			target,
			root: client.root().clone(),
			selection: None,
		})
	}
	pub async fn update_selection(&mut self, origin: Vec3, direction: Vec3) {
		_ = self.query.update(origin.into(), direction.into(), f32::MAX, 0.005);
		self.selection = self.beams.closest().await;
		let Some(selection) = self.selection.clone() else {
			_ = self.selection_lines.set_lines(Vec::new());
			return;
		};
		_ = self.lines_spatial.set_relative_transform(
			selection.spatial.clone(),
			PartialTransform::from_translation_rotation(Vec3::ZERO, Quat::IDENTITY),
		);
		let Ok(Ok(bb)) = self
			.spatial_interface
			.get_relative_bounding_box(self.lines_spatial_ref.clone(), selection.spatial)
			.await
		else {
			warn!("can't get bounding box");
			_ = self.selection_lines.set_lines(Vec::new());
			return;
		};
		let lines = bounding_box(bb)
			.into_iter()
			.map(|l| l.thickness(0.0025))
			.collect::<Vec<_>>();
		_ = self.selection_lines.set_lines(lines);
	}
	pub async fn capture_selected(&mut self) -> Option<CapturedSelection> {
		let selection = self.selection.take()?;
		_ = self.selection_lines.set_lines(Vec::new());

		let bb = self
			.spatial_interface
			.get_relative_bounding_box(selection.spatial.clone(), selection.spatial.clone())
			.await
			.ok()?
			.ok()?;
		let longest = Vec3Component::find_longest(bb.extents);
		let other_size = longest.other_max(bb.extents);
		_ = self.target.set_parent(selection.spatial.clone());
		_ = self
			.target
			.set_local_transform(PartialTransform::from_translation_rotation_scale(
				bb.center,
				longest.rotation() * Quat::from_rotation_y(FRAC_PI_2),
				[other_size * 2.0; 3],
			));

		Some(CapturedSelection {
			selection,
			target: self.target.clone(),
			root: self.root.clone(),
		})
	}
}

#[derive(Debug, Clone)]
pub struct Selection {
	pub spatial: SpatialRef,
	pub poseable: Poseable,
}

pub struct CapturedSelection {
	selection: Selection,
	target: Spatial,
	root: SpatialRef,
}
impl CapturedSelection {
	pub fn spatial(&self) -> &SpatialRef {
		&self.selection.spatial
	}
	pub fn poseable(&self) -> &Poseable {
		&self.selection.poseable
	}
	pub fn dead(&self) -> bool {
		!self.selection.poseable.alive()
	}
}
impl Drop for CapturedSelection {
	fn drop(&mut self) {
		_ = self.target.set_parent(self.root.clone());
		_ = self
			.target
			.set_local_transform(PartialTransform::from_scale(Vec3::ZERO));
	}
}

#[derive(gluon_ipc::Handler)]
struct Beams {
	map: Mutex<FxHashMap<QueryableId, Hit>>,
	interface: SpatialInterface,
	ref_space: SpatialRef,
}
struct Hit {
	selection: Selection,
	bounding_box: BoundingBox,
	min_distance: f32,
	depth: f32,
}
fn count_too_long_axes(vec: impl Into<Vec3>) -> u8 {
	vec.into()
		.to_array()
		.map(|v| if v > MAX_BB_LENGTH { 1 } else { 0 })
		.into_iter()
		.sum()
}
impl Beams {
	async fn closest(&self) -> Option<Selection> {
		self.map
			.lock()
			.await
			.values()
			.filter(|hit| hit.min_distance <= 0.0)
			.filter(|hit| count_too_long_axes(hit.bounding_box.extents) < MAX_TOO_LONG_AXES)
			.reduce(|a, b| if a.depth < b.depth { a } else { b })
			.map(|hit| hit.selection.clone())
	}
	fn poseable(interfaces: Vec<QueriedInterface>) -> Option<Poseable> {
		interfaces
			.into_iter()
			.find(|i| i.interface_id == Poseable::ID)
			.map(|i| Poseable::from_ref(i.interface))
	}
}
impl BeamQueryHandlerHandler for Beams {
	async fn intersected(
		&self,
		_ctx: gluon_ipc::Context,
		obj: QueryableId,
		_field: FieldRef,
		spatial: SpatialRef,
		interfaces: Vec<QueriedInterface>,
		spatial_info: RayMarchResult,
	) {
		let Some(poseable) = Self::poseable(interfaces) else {
			return;
		};
		let Ok(Ok(bb)) = self
			.interface
			.get_relative_bounding_box(self.ref_space.clone(), spatial.clone())
			.await
		else {
			return;
		};
		self.map.lock().await.insert(
			obj,
			Hit {
				selection: Selection { spatial, poseable },
				min_distance: spatial_info.min_distance,
				depth: spatial_info.deepest_point_distance,
				bounding_box: bb,
			},
		);
	}

	async fn interfaces_changed(
		&self,
		_ctx: gluon_ipc::Context,
		obj: QueryableId,
		interfaces: Vec<QueriedInterface>,
	) {
		let mut hits = self.map.lock().await;
		match Self::poseable(interfaces) {
			Some(poseable) => {
				if let Some(hit) = hits.get_mut(&obj) {
					hit.selection.poseable = poseable;
				}
			}
			None => {
				hits.remove(&obj);
			}
		}
	}

	async fn moved(&self, _ctx: gluon_ipc::Context, obj: QueryableId, spatial_info: RayMarchResult) {
		if let Some(hit) = self.map.lock().await.get_mut(&obj) {
			hit.min_distance = spatial_info.min_distance;
			hit.depth = spatial_info.deepest_point_distance;
			let Ok(Ok(bb)) = self
				.interface
				.get_relative_bounding_box(self.ref_space.clone(), hit.selection.spatial.clone())
				.await
			else {
				return;
			};
			hit.bounding_box = bb;
		}
	}

	async fn left(&self, _ctx: gluon_ipc::Context, obj: QueryableId) {
		self.map.lock().await.remove(&obj);
	}
}

enum Vec3Component {
	X,
	Y,
	Z,
}
impl Vec3Component {
	fn find_longest(vec: impl Into<Vec3>) -> Self {
		let v = vec.into();
		if v.x >= v.y && v.x >= v.z {
			Self::X
		} else if v.y >= v.x && v.y >= v.z {
			Self::Y
		} else {
			Self::Z
		}
	}
	fn other_max(&self, vec: impl Into<Vec3>) -> f32 {
		let v = vec.into();
		match self {
			Vec3Component::X => v.y.max(v.z),
			Vec3Component::Y => v.x.max(v.z),
			Vec3Component::Z => v.x.max(v.y),
		}
	}
	fn rotation(&self) -> Quat {
		match self {
			Vec3Component::X => Quat::from_rotation_y(FRAC_PI_2) * Quat::from_rotation_x(FRAC_PI_2),
			Vec3Component::Y => Quat::IDENTITY,
			Vec3Component::Z => Quat::from_rotation_x(FRAC_PI_2),
		}
	}
}

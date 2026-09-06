use glam::{FloatExt, Quat, Vec3};
use stardust_xr_fusion::{
	spatial::{SpatialInterface, SpatialRef},
	types::Posef,
};

use crate::selection::CapturedSelection;

pub struct Mover {
	selection: CapturedSelection,
	spatial_interface: SpatialInterface,
	input: SpatialRef,
	distance: f32,
}

impl Mover {
	pub async fn new(
		selection: CapturedSelection,
		spatial_interface: SpatialInterface,
		input: SpatialRef,
	) -> Self {
		let distance = spatial_interface
			.get_relative_transform(input.clone(), selection.spatial().clone())
			.await
			.ok()
			.and_then(|t| t.ok())
			.map(|t| Vec3::from(t.translation).length())
			.unwrap_or_default();
		Mover {
			selection,
			spatial_interface,
			input,
			distance,
		}
	}
	pub async fn update(&mut self) {
		let Ok(Ok(transform)) = self
			.spatial_interface
			.get_relative_transform(self.input.clone(), self.selection.spatial().clone())
			.await
		else {
			return;
		};
		let sel_translation = Vec3::from(transform.translation);
		let sel_rotation = Quat::from(transform.rotation);
		let lerp_factor = 0.95;
		// the hold point sits straight down -Z at a fixed distance, so lerping away from it
		// is what pulls the thing in over several frames instead of snapping
		let quat = Quat::IDENTITY.slerp(
			Quat::from_rotation_arc(Vec3::NEG_Z, sel_translation.normalize()),
			lerp_factor,
		);
		let len = self.distance.lerp(sel_translation.length(), lerp_factor);
		_ = self.selection.poseable().set_relative_pose(
			self.input.clone(),
			Posef {
				position: ((quat * Vec3::NEG_Z) * len).into(),
				orientation: Quat::IDENTITY.slerp(sel_rotation, lerp_factor).into(),
			},
		);
	}
	pub fn dead(&self) -> bool {
		self.selection.dead()
	}
}

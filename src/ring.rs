use core::f32;
use std::{process, sync::Arc};

use glam::{Quat, Vec3, Vec3A};
use stardust_xr_fusion::{
	Result,
	client::{Client, ClientHandler, FrameInfo},
	drawable::{Line, LinePoint, Lines, LinesExt},
	fields::{Field, FieldExt, Shape},
	spatial::{Spatial, SpatialExt, SpatialRef, Transform},
	suis::{InputDataType, InputMethod},
	types::color::rgba,
};
use stardust_xr_molecules::{
	Derezzable, FrameSensitive, UIElement,
	grabbable::{Grabbable, GrabbableSettings, PointerMode},
	input_action::{InputQueue, InputSnapshot},
	lines::{LineExt, circle},
};

pub struct Ring {
	grabbable: Grabbable,
	derezzable: Derezzable,
	_ring_lines: Lines,
	_input_field: Field,
	pub input: InputQueue,
	input_space: SpatialRef,
	attached_to: Option<InputMethod>,
	attach_lines: Lines,
}
impl Ring {
	pub async fn new(client: &Client<impl ClientHandler>) -> Result<Self> {
		let (spatial, spatial_ref) =
			Spatial::new(client, client.root(), Transform::IDENTITY).await?;
		let grab_radius = 0.05;
		let grab_thickness = 0.005;
		// the ring is drawn and felt on the XZ plane, stood up to face -Z
		let (ring_spatial, _) = Spatial::new(
			client,
			&spatial_ref,
			Transform::from_rotation(Quat::from_rotation_x(f32::consts::FRAC_PI_2)),
		)
		.await?;
		let (grabbable_field, _) = Field::new(
			client,
			&ring_spatial,
			Shape::Torus {
				major_radius: grab_radius,
				minor_radius: grab_thickness,
			},
		)
		.await?;
		let grabbable = Grabbable::new(
			client,
			spatial_ref.clone(),
			Transform::IDENTITY,
			grabbable_field.clone(),
			GrabbableSettings {
				max_distance: 0.03,
				linear_momentum: None,
				angular_momentum: None,
				pointer_mode: PointerMode::Align,
			},
		)
		.await?;
		let content_parent = grabbable.content_parent().spatial_ref().await?;
		ring_spatial.set_parent(content_parent.clone())?;

		let _ring_lines = Lines::new(
			client,
			&ring_spatial,
			vec![circle(32, 0.0, grab_radius).thickness(grab_thickness)],
		)
		.await?;

		let (input_field_spatial, _) = Spatial::new(
			client,
			&content_parent,
			Transform::from_translation([0.0, 0.0, -0.05]),
		)
		.await?;
		let (input_field, _) =
			Field::new(client, &input_field_spatial, Shape::Sphere { radius: 0.1 }).await?;
		let input = InputQueue::new(
			client,
			input_field_spatial,
			input_field.clone(),
			spatial_ref.clone(),
		)
		.await?;

		let attach_lines = Lines::new(client, &spatial, Vec::new()).await?;
		let derezzable = Derezzable::new(client, ring_spatial, grabbable_field).await?;

		Ok(Ring {
			grabbable,
			_ring_lines,
			_input_field: input_field,
			input,
			input_space: spatial_ref,
			attached_to: None,
			attach_lines,
			derezzable,
		})
	}
	/// everything the ring's input is relative to, and what the beam is aimed in
	pub fn input_space(&self) -> &SpatialRef {
		&self.input_space
	}
	pub fn update(&mut self, frame_info: &FrameInfo) {
		if self.derezzable.receiver.try_recv().is_ok() {
			process::exit(0);
		}
		let grab_event = self.grabbable.handle_events();
		let input_event = self.input.handle_events();
		if !(grab_event || input_event) {
			return;
		}
		self.grabbable.frame(frame_info);
		if self.grabbable.grab_action().actor_started() && self.attached_to.is_some() {
			self.on_detach();
		}

		let pos = self.grabbable.pose().0.into();
		let attaching_to = self.get_input_to_capture(pos);
		if self.grabbable.grab_action().actor_acting()
			&& let Some(attaching_to) = attaching_to.as_ref()
		{
			let point = match attaching_to.input() {
				InputDataType::Pointer { .. } => Vec3::ZERO,
				InputDataType::Tip { data: tip } => {
					let quat = Quat::from(tip.pose.orientation);
					Vec3::from(tip.pose.position) + quat.mul_vec3(Vec3::Z * 0.05)
				}
				InputDataType::Hand { data: hand } => Vec3::from(hand.wrist.pose.position),
			};
			_ = self.attach_lines.set_lines(vec![Line {
				points: vec![
					LinePoint {
						point: point.into(),
						thickness: 0.005,
						color: rgba!(0.7, 0.7, 0.7, 1.0).to_linear(),
					},
					LinePoint {
						point: pos.into(),
						thickness: 0.005,
						color: rgba!(0.7, 0.7, 0.7, 1.0).to_linear(),
					},
				],
				cyclic: false,
			}]);
		} else {
			_ = self.attach_lines.set_lines(Vec::new());
		}
		if self.grabbable.grab_action().actor_stopped()
			&& let Some(attaching_to) = attaching_to
		{
			self.on_attach(&attaching_to);
		}
		if let Some(input) = self.get_attached_input() {
			let (pos, rot) = match input.input() {
				InputDataType::Pointer { .. } => (Vec3::ZERO, Quat::IDENTITY),
				InputDataType::Tip { data: tip } => {
					let quat = Quat::from(tip.pose.orientation);
					(
						Vec3::from(tip.pose.position) + quat.mul_vec3(Vec3::Z * 0.05),
						quat,
					)
				}
				InputDataType::Hand { data: hand } => (
					hand.wrist.pose.position.into(),
					Quat::from(hand.wrist.pose.orientation),
				),
			};
			self.grabbable.set_pose(pos, rot);
		}
	}
	fn on_attach(&mut self, snap: &InputSnapshot) {
		self.input.start_capture(snap);
		self.attached_to = Some(snap.method.clone());
	}
	fn on_detach(&mut self) {
		if let Some(snap) = self.get_attached_input() {
			self.input.release_capture(&snap);
		}
		self.attached_to.take();
	}
	pub fn get_attached_input(&self) -> Option<Arc<InputSnapshot>> {
		let attached = self.attached_to.as_ref()?;
		self.input.input().get(attached).cloned()
	}
	fn get_input_to_capture(&self, pos: Vec3A) -> Option<Arc<InputSnapshot>> {
		self.input
			.input()
			.into_values()
			.filter(|snap| match snap.input() {
				InputDataType::Pointer { .. } => false,
				InputDataType::Tip { data: tip } => {
					let quat = Quat::from(tip.pose.orientation);
					pos.distance(Vec3A::from(tip.pose.position) + quat.mul_vec3a(Vec3A::Z * 0.05))
						< 0.05
				}
				InputDataType::Hand { data: hand } => {
					pos.distance(hand.wrist.pose.position.into()) < 0.05
				}
			})
			.reduce(|a, b| if a.distance() < b.distance() { a } else { b })
	}
}

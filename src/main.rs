pub mod mover;
pub mod ring;
pub mod selection;

use std::f32::consts::FRAC_PI_2;

use glam::{Quat, Vec3};
use gluon::Liveness;
use stardust_xr_fusion::{
	client::Client,
	drawable::{Line, LinePoint, Lines, LinesExt, MaterialParameter, Model, ModelExt},
	project_local_resources,
	spatial::{PartialTransform, Spatial, SpatialExt, Transform},
	suis::{Chirality, InputDataType},
	types::{Resource, rgba_linear},
};
use stardust_xr_molecules::{accent_color::AccentColor, input_action::SimpleAction};
use tokio::sync::broadcast::error::RecvError;
use zbus::Connection;

use crate::{mover::Mover, ring::Ring, selection::Selector};

pub const APP_ID: &str = "absolute_solver";

#[tokio::main]
async fn main() {
	tracing_subscriber::fmt().init();
	let (client, root) = Client::connect(&[&project_local_resources!("res")])
		.await
		.unwrap();
	let mut accent_color = AccentColor::new(Connection::session().await.unwrap());
	let mut ring = Ring::new(&client).await.unwrap();

	let (debug_spatial, _) = Spatial::new(&client, ring.input_space(), Transform::IDENTITY)
		.await
		.unwrap();
	let lines = Lines::new(&client, &debug_spatial, Vec::new())
		.await
		.unwrap();

	let (input_spatial, input_spatial_ref) =
		Spatial::new(&client, ring.input_space(), Transform::IDENTITY)
			.await
			.unwrap();
	let mut captured_selection: Option<Mover> = None;

	let mut solver_active = SimpleAction::default();
	let (solver_spatial, _) = Spatial::new(&client, &root, Transform::from_scale(Vec3::ZERO))
		.await
		.unwrap();
	let solver_model = Model::new(&client, &solver_spatial, solver_resource())
		.await
		.unwrap();

	let (target_spatial, _) = Spatial::new(&client, &root, Transform::from_scale(Vec3::ZERO))
		.await
		.unwrap();
	let target_model = Model::new(&client, &target_spatial, solver_resource())
		.await
		.unwrap();

	let mut selector = Selector::new(&client, ring.input_space().clone(), target_spatial)
		.await
		.unwrap();

	let solver_part = solver_model.get_part("Solver").await.unwrap().unwrap();
	let target_part = target_model.get_part("Solver").await.unwrap().unwrap();
	tokio::task::spawn(async move {
		while accent_color.color.changed().await.is_ok() {
			let mut color = accent_color.color();

			// bad hack so we can get a max value color
			let greatest_channel = color.c.r.max(color.c.g).max(color.c.b);
			let factor = 1.0 / greatest_channel;
			color.c.r *= factor;
			color.c.g *= factor;
			color.c.b *= factor;

			for part in [&solver_part, &target_part] {
				_ = part
					.set_material_parameter(
						"emission_factor",
						MaterialParameter::Color { value: color },
					)
					.await;
			}
		}
	});

	let mut frames = client.frame_receiver();
	loop {
		let frame_info = tokio::select! {
			frame = frames.recv() => match frame {
				Ok(info) => info,
				Err(RecvError::Lagged(_)) => continue,
				Err(RecvError::Closed) => break,
			},
			_ = client.server().death_notification() => break,
		};
		ring.update(&frame_info);

		let Some(input) = ring.get_attached_input() else {
			_ = lines.set_lines(Vec::new());
			_ = solver_spatial.set_local_transform(PartialTransform::from_scale(Vec3::ZERO));
			_ = captured_selection.take();
			continue;
		};
		solver_active.update(&ring.input, &|data| match data.input() {
			InputDataType::Pointer { .. } => false,
			InputDataType::Hand { data: hand } => {
				let distance = Vec3::from(hand.thumb.tip.pose.position)
					.distance(hand.index.tip.pose.position.into())
					- (hand.thumb.tip.radius + hand.index.tip.radius);

				distance > 0.02
			}
			InputDataType::Tip { .. } => data.datamap_f32("grab") > 0.5,
		});

		let mut lines_data = Vec::new();
		let (triangle_center, rotation, diameter, selection_dir) = match input.input() {
			InputDataType::Tip { data: tip } => (
				tip.pose.position.into(),
				tip.pose.orientation.into(),
				0.1,
				Quat::from(tip.pose.orientation) * Vec3::NEG_Z,
			),
			InputDataType::Hand { data: hand } => {
				let mut p: [Vec3; 3] = [
					hand.thumb.tip.pose.position.into(),
					hand.index.tip.pose.position.into(),
					hand.middle.tip.pose.position.into(),
				];
				if hand.chirality == Chirality::Left {
					p.reverse();
				}
				lines_data.push(Line {
					points: p
						.iter()
						.copied()
						.map(|p| LinePoint {
							point: p.into(),
							thickness: 0.001,
							color: rgba_linear!(1.0, 0.0, 1.0, 1.0),
						})
						.collect(),
					cyclic: true,
				});
				let (position, rotation) =
					get_position_and_normal_from_triangle(p, hand.palm.pose.orientation.into());
				let max_distance_from_center = p
					.iter()
					.map(|point| point.distance(position))
					.reduce(|a, b| if a > b { a } else { b })
					.unwrap_or_default();
				let palm = Vec3::from(hand.palm.pose.position);
				(
					position,
					rotation,
					max_distance_from_center * 2.0,
					(position - palm).normalize(),
				)
			}
			_ => {
				continue;
			}
		};
		let normal = rotation * Vec3::NEG_Z;
		lines_data.push(Line {
			points: vec![
				LinePoint {
					point: triangle_center.into(),
					thickness: 0.001,
					color: rgba_linear!(0.0, 1.0, 0.0, 1.0),
				},
				LinePoint {
					point: (triangle_center + (normal * 0.01)).into(),
					thickness: 0.001,
					color: rgba_linear!(0.0, 1.0, 0.0, 1.0),
				},
			],
			cyclic: false,
		});
		lines_data.push(Line {
			points: vec![
				LinePoint {
					point: triangle_center.into(),
					thickness: 0.001,
					color: rgba_linear!(0.0, 0.0, 1.0, 1.0),
				},
				LinePoint {
					point: (triangle_center + (selection_dir * 0.01)).into(),
					thickness: 0.001,
					color: rgba_linear!(0.0, 0.0, 1.0, 1.0),
				},
			],
			cyclic: false,
		});
		lines.set_lines(lines_data).unwrap();

		_ = input_spatial.set_local_transform(PartialTransform::from_translation_rotation(
			triangle_center,
			{
				let ref_quat = rotation;
				ref_quat * Quat::from_rotation_arc(Vec3::NEG_Z, ref_quat.inverse() * normal)
			},
		));

		if solver_active.started_acting().contains(&input) {
			captured_selection = match selector.capture_selected().await {
				Some(sel) => Some(
					Mover::new(
						sel,
						client.spatial_interface().clone(),
						input_spatial_ref.clone(),
					)
					.await,
				),
				None => None,
			};
		}
		// we can use this solver active with containing input to get when we start and stop expanding our fingers to be able to switch between selection and levitation
		if solver_active.currently_acting().contains(&input) {
			if let Some(sel) = captured_selection.as_mut() {
				sel.update().await;
			};
			let scale = if captured_selection.is_some() {
				diameter * 2.0
			} else {
				0.0
			};
			_ = solver_spatial.set_relative_transform(
				ring.input_space().clone(),
				PartialTransform::from_translation_rotation_scale(
					triangle_center + (normal * 0.01),
					rotation * Quat::from_rotation_x(FRAC_PI_2),
					[scale; 3],
				),
			);
		} else {
			captured_selection.take();
			_ = solver_spatial.set_local_transform(PartialTransform::from_scale(Vec3::ZERO));
			selector
				.update_selection(triangle_center, selection_dir)
				.await;
		}
	}
}

fn solver_resource() -> Resource {
	Resource::Namespaced {
		namespace: APP_ID.into(),
		path: "solver".into(),
	}
}

fn get_position_and_normal_from_triangle(points: [Vec3; 3], ref_quat: Quat) -> (Vec3, Quat) {
	let [a, b, c] = points;
	let ab = a.distance_squared(b);
	let bc = b.distance_squared(c);
	let ca = c.distance_squared(a);
	let point_a = ((bc * a) + (ca * b) + (ab * c)) / (ab + bc + ca);
	let a_dist = a.distance_squared(point_a);
	let b_dist = b.distance_squared(point_a);
	let c_dist = c.distance_squared(point_a);
	let point = ((a_dist * a) + (b_dist * b) + (c_dist * c)) / (a_dist + b_dist + c_dist);
	let point = point.lerp(point_a, 0.5);
	let ab = b - a;
	let ac = c - a;
	let normal = ab.cross(ac).normalize();
	(
		point,
		ref_quat * Quat::from_rotation_arc(Vec3::NEG_Z, ref_quat.inverse() * normal),
	)
}

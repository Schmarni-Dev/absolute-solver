pub mod mover;
pub mod ring;
pub mod selection;

use std::f32::consts::FRAC_PI_2;

use glam::{Quat, Vec3};
use stardust_xr_fusion::{
    client::Client,
    core::schemas::zbus::Connection,
    drawable::{Line, LinePoint, Lines, LinesAspect, MaterialParameter, Model, ModelPartAspect},
    input::InputDataType,
    node::NodeType,
    objects::object_registry::ObjectRegistry,
    project_local_resources,
    root::{RootAspect, RootEvent},
    spatial::{Spatial, SpatialAspect, Transform},
    values::{ResourceID, color::rgba_linear},
};
use stardust_xr_molecules::{accent_color::AccentColor, input_action::SimpleAction};

use crate::{
    mover::Mover,
    ring::Ring,
    selection::{Ray, Selector},
};

#[tokio::main]
async fn main() {
    tracing_subscriber::fmt().init();
    let client = Client::connect().await.unwrap();
    client
        .setup_resources(&[&project_local_resources!("res")])
        .unwrap();
    let event_loop = client.async_event_loop();
    let client = event_loop.client_handle.clone();
    let lines = Lines::create(client.get_root(), Transform::none(), &[]).unwrap();
    let conn = Connection::session().await.unwrap();
    let obj_reg = ObjectRegistry::new(&conn).await;
    let mut accent_color = AccentColor::new(conn.clone());
    let mut ring = Ring::new(conn, &client).unwrap();

    let input_spatial = Spatial::create(client.get_root(), Transform::none(), false).unwrap();
    let mut captured_selection: Option<Mover> = None;

    let mut solver_active = SimpleAction::default();
    let solver_model = Model::create(
        client.get_root(),
        Transform::identity(),
        &ResourceID::new_namespaced("absolute_solver", "solver"),
    )
    .unwrap();

    let solver_target_model = Model::create(
        client.get_root(),
        Transform::identity(),
        &ResourceID::new_namespaced("absolute_solver", "solver"),
    )
    .unwrap();
    _ = solver_target_model.set_enabled(false);

    let mut selector = Selector::new(client.clone(), obj_reg, solver_target_model.clone())
        .await
        .unwrap();

    // change solver color to match accent color
    let solver_part = solver_model.part("Solver").unwrap();
    let solver_target_part = solver_target_model.part("Solver").unwrap();
    tokio::task::spawn(async move {
        while accent_color.color.changed().await.is_ok() {
            let mut color = accent_color.color();

            // bad hack so we can get a max value color
            let greatest_channel = color.c.r.max(color.c.g).max(color.c.b);
            let factor = 1.0 / greatest_channel;
            color.c.r *= factor;
            color.c.g *= factor;
            color.c.b *= factor;

            solver_part
                .set_material_parameter("emission_factor", MaterialParameter::Color(color))
                .unwrap();
            solver_target_part
                .set_material_parameter("emission_factor", MaterialParameter::Color(color))
                .unwrap();
        }
    });

    loop {
        event_loop.get_event_handle().wait().await;
        let Some(event) = client.get_root().recv_root_event() else {
            continue;
        };
        let frame_info = match event {
            RootEvent::Ping { response } => {
                response.send_ok(());
                continue;
            }
            RootEvent::Frame { info } => info,
            RootEvent::SaveState { response: _ } => {
                break;
            }
        };
        ring.update(&frame_info);

        let Some(input) = ring.get_attached_input() else {
            _ = lines.set_lines(&[]);
            _ = solver_model.set_enabled(false);
            continue;
        };
        solver_active.update(&ring.input, &|data| match &data.input {
            InputDataType::Pointer(_) => false,
            InputDataType::Hand(hand) => {
                let p: [Vec3; 3] = [
                    hand.thumb.tip.position.into(),
                    hand.index.tip.position.into(),
                    hand.middle.tip.position.into(),
                ];
                let (center, _) =
                    get_position_and_normal_from_triangle(p, hand.palm.rotation.into());
                let max_distance_from_center = p
                    .iter()
                    .map(|point| point.distance(center))
                    .reduce(|a, b| if a < b { a } else { b })
                    .unwrap_or_default();

                max_distance_from_center > 0.025
            }
            InputDataType::Tip(_) => data.datamap.with_data(|d| d.idx("grab").as_f32() > 0.5),
        });

        let mut lines_data = Vec::new();
        let (triangle_center, rotation, diameter, selection_dir) = match &input.input {
            InputDataType::Tip(tip) => (
                tip.origin.into(),
                tip.orientation.into(),
                0.1,
                Quat::from(tip.orientation) * Vec3::NEG_Z,
            ),
            InputDataType::Hand(hand) => {
                let mut p: [Vec3; 3] = [
                    hand.thumb.tip.position.into(),
                    hand.index.tip.position.into(),
                    hand.middle.tip.position.into(),
                ];
                if !hand.right {
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
                    get_position_and_normal_from_triangle(p, hand.palm.rotation.into());
                let max_distance_from_center = p
                    .iter()
                    .map(|point| point.distance(position))
                    .reduce(|a, b| if a > b { a } else { b })
                    .unwrap_or_default();
                let palm = Vec3::from(hand.palm.position);
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
        lines.set_lines(&lines_data).unwrap();

        _ = input_spatial.set_local_transform(Transform::from_translation_rotation(
            triangle_center,
            {
                let ref_quat = rotation;
                ref_quat * Quat::from_rotation_arc(Vec3::NEG_Z, ref_quat.inverse() * normal)
            },
        ));

        if solver_active.started_acting().contains(&input) {
            let sel = selector.capture_selected().await;
            captured_selection = match sel {
                Some(sel) => Mover::new(sel, input_spatial.clone().as_spatial_ref())
                    .await
                    .ok(),
                None => None,
            };
        }
        // we can use this solver active with containing input to get when we start and stop expanding our fingers to be able to switch between selection and levitation
        if solver_active.currently_acting().contains(&input) {
            // TODO: replace with actual transform functionality
            if let Some(sel) = captured_selection.as_mut() {
                sel.update().await;
            };
            solver_model.set_enabled(true).unwrap();
            solver_model
                .set_local_transform(Transform::from_translation_rotation_scale(
                    triangle_center + (normal * 0.01),
                    rotation * Quat::from_rotation_x(FRAC_PI_2),
                    [diameter * 2.0; 3],
                ))
                .unwrap();
        } else {
            captured_selection.take();
            solver_model.set_enabled(false).unwrap();
            selector
                .update_selection(Ray {
                    origin: triangle_center,
                    direction: selection_dir,
                    ref_space: ring.input.handler().clone().as_spatial_ref(),
                })
                .await;
        }
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

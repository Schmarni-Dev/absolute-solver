pub mod ring;

use std::f32::consts::FRAC_PI_2;

use glam::{Quat, Vec3};
use stardust_xr_fusion::{
    client::Client,
    core::schemas::zbus::Connection,
    drawable::{Line, LinePoint, Lines, LinesAspect, Model},
    input::InputDataType,
    node::{NodeResult, OwnedAspect},
    project_local_resources,
    root::{RootAspect, RootEvent},
    spatial::{SpatialAspect, Transform},
    values::{ResourceID, color::rgba_linear},
};
use stardust_xr_molecules::input_action::SimpleAction;

use crate::ring::Ring;

#[tokio::main]
async fn main() -> NodeResult<()> {
    let client = Client::connect().await.unwrap();
    client
        .setup_resources(&[&project_local_resources!("res")])
        .unwrap();
    let event_loop = client.async_event_loop();
    let client = event_loop.client_handle.clone();
    let lines = Lines::create(client.get_root(), Transform::none(), &[])?;
    let conn = Connection::session().await.unwrap();
    let mut ring = Ring::new(conn, &client)?;

    let mut solver_active = SimpleAction::default();

    let solver = Model::create(
        client.get_root(),
        Transform::identity(),
        &ResourceID::new_namespaced("absolute_solver", "solver"),
    )?;

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
                let (center, _) = get_position_and_normal_from_triangle(p);
                let max_distance_from_center = p
                    .iter()
                    .map(|point| point.distance(center))
                    .reduce(|a, b| if a > b { a } else { b })
                    .unwrap_or_default();

                max_distance_from_center > 0.025
            }
            InputDataType::Tip(_) => data.datamap.with_data(|d| d.idx("grab").as_f32() > 0.5),
        });

        let mut lines_data = Vec::new();
        let (triangle_center, rotation, diameter) = match &input.input {
            InputDataType::Tip(tip) => (tip.origin.into(), tip.orientation.into(), 0.005),
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
                let (position, rotation) = get_position_and_normal_from_triangle(p);
                let max_distance_from_center = p
                    .iter()
                    .map(|point| point.distance(position))
                    .reduce(|a, b| if a > b { a } else { b })
                    .unwrap_or_default();
                (position, rotation, max_distance_from_center * 2.0)
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
        lines.set_lines(&lines_data)?;
        // we can use this solver active with containing input to get when we start and stop expanding our fingers to be able to switch between selection and levitation
        if solver_active.currently_acting().contains(&input) {
            solver.set_enabled(true)?;
            solver.set_local_transform(Transform::from_translation_rotation_scale(
                triangle_center,
                rotation * Quat::from_rotation_x(FRAC_PI_2),
                [diameter * 2.0; 3],
            ))?;
        } else {
            solver.set_enabled(false)?;
        }
    }
    Ok(())
}

fn get_position_and_normal_from_triangle(points: [Vec3; 3]) -> (Vec3, Quat) {
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
    (point, Quat::from_rotation_arc(Vec3::NEG_Z, normal))
}

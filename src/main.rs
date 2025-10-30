pub mod ring;

use glam::{Quat, Vec3};
use stardust_xr_fusion::{
    client::Client,
    core::schemas::zbus::Connection,
    drawable::{Line, LinePoint, Lines, LinesAspect},
    input::InputDataType,
    node::NodeResult,
    root::{RootAspect, RootEvent},
    spatial::Transform,
    values::color::rgba_linear,
};

use crate::ring::Ring;

#[tokio::main]
async fn main() -> NodeResult<()> {
    let client = Client::connect().await.unwrap();
    let event_loop = client.async_event_loop();
    let client = event_loop.client_handle.clone();
    let lines = Lines::create(client.get_root(), Transform::none(), &[])?;
    let conn = Connection::session().await.unwrap();
    let mut ring = Ring::new(conn, &client)?;

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
        let mut lines_data = Vec::new();
        let (point, rotation) = match &input.input {
            InputDataType::Tip(tip) => (tip.origin.into(), tip.orientation.into()),
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
                get_position_and_normal_from_triangle(p)
            }
            _ => {
                continue;
            }
        };
        let normal = rotation * Vec3::NEG_Z;
        lines_data.push(Line {
            points: vec![
                LinePoint {
                    point: point.into(),
                    thickness: 0.001,
                    color: rgba_linear!(0.0, 1.0, 0.0, 1.0),
                },
                LinePoint {
                    point: (point + (normal * 0.01)).into(),
                    thickness: 0.001,
                    color: rgba_linear!(0.0, 1.0, 0.0, 1.0),
                },
            ],
            cyclic: false,
        });
        lines.set_lines(&lines_data)?;
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

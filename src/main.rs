pub mod ring;

use std::sync::Mutex;

use glam::Vec3;
use stardust_xr_fusion::{
    client::Client, core::schemas::zbus::Connection, drawable::{Line, LinePoint, Lines, LinesAspect}, fields::{Field, Shape}, input::InputHandler, node::NodeResult, root::{RootAspect, RootEvent}, spatial::Transform, values::color::rgba_linear
};
use stardust_xr_molecules::input_action::{InputQueueable, MultiAction};

use crate::ring::Ring;

#[tokio::main]
async fn main() -> NodeResult<()> {
    let client = Client::connect().await.unwrap();
    let event_loop = client.async_event_loop();
    let client = event_loop.client_handle.clone();
    let field = Field::create(client.get_root(), Transform::none(), Shape::Sphere(1.0))?;
    let mut input_handler =
        InputHandler::create(client.get_root(), Transform::none(), &field)?.queue()?;
    let mut action = MultiAction::default();
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
        

        let points: Mutex<Vec<[Vec3; 3]>> = Mutex::new(Vec::new());
        input_handler.handle_events();
        action.update(
            &input_handler,
            |_| true,
            |v| match &v.input {
                stardust_xr_fusion::input::InputDataType::Pointer(_) => false,
                stardust_xr_fusion::input::InputDataType::Hand(hand) => {
                    let mut points = points.lock().unwrap();
                    let mut p = [
                        hand.thumb.tip.position.into(),
                        hand.index.tip.position.into(),
                        hand.middle.tip.position.into(),
                    ];
                    if !hand.right {
                        p.reverse();
                    }
                    points.push(p);
                    false
                }
                stardust_xr_fusion::input::InputDataType::Tip(_) => false,
            },
        );
        let mut lines_data = Vec::new();
        for points in points.lock().unwrap().clone() {
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
            lines_data.push(Line {
                points: points
                    .into_iter()
                    .map(|p| LinePoint {
                        point: p.into(),
                        thickness: 0.001,
                        color: rgba_linear!(1.0, 0.0, 1.0, 1.0),
                    })
                    .collect(),
                cyclic: true,
            });
            let ab = b - a;
            let ac = c - a;
            let normal = ab.cross(ac).normalize();
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
        }
        lines.set_lines(&lines_data)?;
    }
    Ok(())
}

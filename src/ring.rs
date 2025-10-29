use core::f32;
use std::sync::Arc;

use glam::{Quat, Vec3, Vec3A};
use stardust_xr_fusion::{
    ClientHandle,
    core::schemas::zbus::Connection,
    drawable::{Line, LinePoint, Lines, LinesAspect},
    fields::{Field, Shape},
    input::{InputData, InputDataType, InputHandler, InputMethodRef, InputMethodRefAspect},
    node::NodeResult,
    root::FrameInfo,
    spatial::{Spatial, SpatialAspect, Transform},
    values::color::rgba,
};
use stardust_xr_molecules::{
    FrameSensitive, Grabbable, GrabbableSettings, PointerMode, UIElement,
    input_action::{InputQueue, InputQueueable},
    lines::{LineExt, circle},
};

pub struct Ring {
    grabbable: Grabbable,
    _grabbable_lines: Lines,
    _input_field: Field,
    input: InputQueue,
    attached_to: Option<InputMethodRef>,
    attach_lines: Lines,
}
impl Ring {
    pub fn new(conn: Connection, client: &Arc<ClientHandle>) -> NodeResult<Self> {
        let spatial = Spatial::create(client.get_root(), Transform::none(), false)?;
        let grab_radius = 0.05;
        let grab_thickness = 0.005;
        let grabbable_shape = Shape::Torus(stardust_xr_fusion::fields::TorusShape {
            radius_b: grab_radius,
            radius_a: grab_thickness,
        });
        let grabbable_field = Field::create(
            &spatial,
            Transform::from_rotation(Quat::from_rotation_x(f32::consts::FRAC_PI_2)),
            grabbable_shape.clone(),
        )?;
        let grabbable = Grabbable::create(
            conn,
            "/Ring",
            &spatial,
            Transform::none(),
            &grabbable_field,
            GrabbableSettings {
                max_distance: 0.01,
                linear_momentum: None,
                angular_momentum: None,
                magnet: false,
                pointer_mode: PointerMode::Align,
                reparentable: true,
            },
        )?;
        let grabbable_spatial = grabbable.content_parent();
        grabbable_field.set_spatial_parent(&grabbable_spatial)?;
        let _grabbable_lines = Lines::create(
            &grabbable_spatial,
            Transform::from_rotation(Quat::from_rotation_x(f32::consts::FRAC_PI_2)),
            &[circle(32, 0.0, grab_radius).thickness(grab_thickness)],
        )?;
        let input_field = Field::create(
            &grabbable_spatial,
            Transform::from_translation([0.0, 0.0, -0.05]),
            Shape::Sphere(0.1),
        )?;
        let input = InputHandler::create(&spatial, Transform::none(), &input_field)?.queue()?;
        let attach_lines = Lines::create(&spatial, Transform::none(), &[])?;
        Ok(Ring {
            grabbable,
            _grabbable_lines,
            _input_field: input_field,
            input,
            attached_to: None,
            attach_lines,
        })
    }
    pub fn update(&mut self, frame_info: &FrameInfo) {
        if !self.grabbable.handle_events() {
            return;
        }
        if !self.input.handle_events() {
            return;
        }
        self.grabbable.frame(frame_info);
        if self.grabbable.grab_action().actor_started() && self.attached_to.is_some() {
            self.on_detach();
        }

        let pos = self.grabbable.pose().0.into();
        let attaching_to = self.get_input_to_capture(pos);
        if self.grabbable.grab_action().actor_acting()
            && let Some((attaching_to, _)) = attaching_to.as_ref()
        {
            let point = match &attaching_to.input {
                InputDataType::Pointer(_) => Vec3::ZERO,
                InputDataType::Tip(tip) => {
                    let quat = Quat::from(tip.orientation);
                    Vec3::from(tip.origin) + quat.mul_vec3(Vec3::Z * 0.05)
                }
                InputDataType::Hand(hand) => Vec3::from(hand.wrist.position),
            };
            _ = self.attach_lines.set_lines(&[Line {
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
            _ = self.attach_lines.set_lines(&[]);
        }
        if self.grabbable.grab_action().actor_stopped()
            && let Some((_, method_ref)) = attaching_to
        {
            self.on_attach(method_ref.clone());
        }
        if let Some(input) = self.get_attached_input() {
            let (pos, rot) = match &input.input {
                InputDataType::Pointer(_) => (Vec3::ZERO, Quat::IDENTITY),
                InputDataType::Tip(tip) => {
                    let quat = Quat::from(tip.orientation);
                    (Vec3::from(tip.origin) + quat.mul_vec3(Vec3::Z * 0.05), quat)
                }
                InputDataType::Hand(hand) => {
                    (hand.wrist.position.into(), hand.wrist.rotation.into())
                }
            };
            self.grabbable.set_pose(pos, rot);
        }
    }
    fn on_attach(&mut self, method_ref: InputMethodRef) {
        if method_ref.try_capture(self.input.handler()).is_ok() {
            self.attached_to = Some(method_ref);
        }
    }
    fn on_detach(&mut self) {
        if let Some(method_ref) = self.attached_to.take() {
            _ = method_ref.release(self.input.handler());
        }
    }
    fn get_attached_input(&self) -> Option<Arc<InputData>> {
        self.attached_to
            .as_ref()
            .and_then(|attached| {
                self.input
                    .input()
                    .into_iter()
                    .find(|(_, method)| *method == attached)
            })
            .map(|(i, _)| i)
    }
    fn get_input_to_capture(&self, pos: Vec3A) -> Option<(Arc<InputData>, &InputMethodRef)> {
        self.input
            .input()
            .into_iter()
            .filter(|(i, _)| match &i.input {
                InputDataType::Pointer(_) => false,
                InputDataType::Tip(tip) => {
                    let quat = Quat::from(tip.orientation);
                    pos.distance(Vec3A::from(tip.origin) + quat.mul_vec3a(Vec3A::Z * 0.05)) < 0.05
                }
                InputDataType::Hand(hand) => pos.distance(hand.wrist.position.into()) < 0.05,
            })
            .reduce(|a, b| if a.0.distance < b.0.distance { a } else { b })
    }
}

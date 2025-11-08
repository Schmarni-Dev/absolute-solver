use glam::{FloatExt, Quat, Vec3, Vec3A};
use stardust_xr_fusion::{
    node::NodeResult,
    spatial::{Spatial, SpatialAspect, SpatialRef, SpatialRefAspect, Transform},
};

use crate::selection::CapturedSelection;

pub struct Mover {
    selection: CapturedSelection,
    target: Spatial,
    input: SpatialRef,
    // m/s
    // selection_velocity: Vec3A,
    // rotation axis scaled by radians/s
    // selection_angular_velocity: Vec3A,
}

impl Mover {
    pub async fn new(selection: CapturedSelection, input_spatial: SpatialRef) -> NodeResult<Self> {
        let target = Spatial::create(&input_spatial, Transform::none(), false)?;
        let len = selection
            .spatial()
            .get_transform(&input_spatial)
            .await?
            .translation
            .map(Vec3A::from)
            .unwrap_or_default()
            .length();
        _ = target.set_local_transform(Transform::from_translation(Vec3::NEG_Z * len));
        Ok(Self {
            selection,
            target,
            input: input_spatial,
        })
    }
    pub async fn update(&mut self) {
        let sel = self.selection.spatial();
        let sel_transform = sel.get_transform(&self.input).await.unwrap();
        let target_transform = self.target.get_transform(&self.input).await.unwrap();
        let sel_translation = sel_transform
            .translation
            .map(Vec3A::from)
            .unwrap_or_default();
        let sel_rotation = sel_transform.rotation.map(Quat::from).unwrap_or_default();
        let target_translation = target_transform
            .translation
            .map(Vec3A::from)
            .unwrap_or_default();
        let target_rotation = target_transform
            .rotation
            .map(Quat::from)
            .unwrap_or_default();
        let lerp_factor = 0.95;
        let sel_len = sel_translation.length();
        let target_len = target_translation.length();
        let sel_quat = Quat::from_rotation_arc(Vec3::NEG_Z, sel_translation.normalize().into());
        let target_quat =
            Quat::from_rotation_arc(Vec3::NEG_Z, target_translation.normalize().into());
        let quat = target_quat.slerp(sel_quat, lerp_factor);
        let len = target_len.lerp(sel_len, lerp_factor);
        let translation = (quat * Vec3::NEG_Z) * len;
        let rotation = target_rotation.slerp(sel_rotation, lerp_factor);
        sel.set_relative_transform(
            &self.input,
            Transform::from_translation_rotation_scale(translation, rotation, Vec3::ONE),
        )
        .unwrap();
    }
}

use glam::{Quat, Vec3, Vec3A};
use stardust_xr_fusion::{
    node::NodeResult,
    spatial::{Spatial, SpatialAspect, SpatialRef, SpatialRefAspect, Transform},
};
use tracing::info;

use crate::selection::CapturedSelection;

pub struct Mover {
    selection: CapturedSelection,
    target: Spatial,
    _input: SpatialRef,
    // m/s
    // selection_velocity: Vec3A,
    // rotation axis scaled by radians/s
    // selection_angular_velocity: Vec3A,
}

impl Mover {
    pub fn new(selection: CapturedSelection, input_spatial: SpatialRef) -> NodeResult<Self> {
        let target = Spatial::create(&input_spatial, Transform::none(), false)?;
        target.set_relative_transform(
            selection.spatial(),
            Transform {
                translation: Some(Vec3::ZERO.into()),
                rotation: Some(Quat::IDENTITY.into()),
                scale: None,
            },
        )?;
        Ok(Self {
            selection,
            target,
            _input: input_spatial,
        })
    }
    pub async fn update(&mut self) {
        let sel = self.selection.spatial();
        let transform = sel.get_transform(&self.target).await.unwrap();
        let pre_translation = transform.translation.map(Vec3A::from).unwrap_or_default();
        let pre_rotation = transform.rotation.map(Quat::from).unwrap_or_default();
        let lerp_factor = 0.95;
        let translation = Vec3A::ZERO.lerp(pre_translation, lerp_factor);
        let rotation = Quat::IDENTITY.lerp(pre_rotation, lerp_factor);
        sel.set_relative_transform(
            &self.target,
            Transform::from_translation_rotation_scale(translation, rotation, Vec3::ONE),
        )
        .unwrap();
    }
}

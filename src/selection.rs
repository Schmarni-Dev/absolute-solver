use core::f32;
use std::{f32::consts::FRAC_PI_2, ops::Deref, sync::Arc};

use glam::{Quat, Vec3};
use stardust_xr_fusion::{
    ClientHandle,
    drawable::{Lines, LinesAspect, Model},
    fields::{FieldRef, FieldRefAspect},
    list_query::{ListEvent, ObjectListQuery},
    node::{NodeResult, NodeType},
    objects::{
        interfaces::{ReparentLockProxy, ReparentableProxy},
        object_registry::ObjectRegistry,
    },
    query::ObjectQuery,
    spatial::{Spatial, SpatialAspect, SpatialRef, SpatialRefAspect, Transform},
};
use stardust_xr_molecules::{
    dbus::AbortOnDrop,
    lines::{LineExt, bounding_box},
};
use tracing::warn;

pub struct Selector {
    query: ObjectListQuery<(
        SpatialRef,
        ReparentableProxy<'static>,
        ReparentLockProxy<'static>,
        Option<FieldRef>,
    )>,
    selection_lines: Lines,
    selection: Option<(
        SpatialRef,
        ReparentableProxy<'static>,
        ReparentLockProxy<'static>,
        Option<FieldRef>,
    )>,
    target_model: Model,
    _mapper_task: AbortOnDrop,
}

impl Selector {
    pub async fn new(
        client: Arc<ClientHandle>,
        object_registry: Arc<ObjectRegistry>,
        target_model: Model,
    ) -> NodeResult<Self> {
        let selection_lines = Lines::create(client.get_root(), Transform::none(), &[])?;
        let (query, mapper) = ObjectQuery::<
            (
                SpatialRef,
                ReparentableProxy<'static>,
                ReparentLockProxy<'static>,
                Option<FieldRef>,
            ),
            ClientHandle,
        >::new(object_registry, client)
        .to_list_query();
        let mapper = tokio::spawn(mapper.init(async |e| match e {
            ListEvent::NewMatch(v) => Some(v),
            ListEvent::Modified(v) => Some(v),
            ListEvent::MatchLost => None,
            _ => None,
        }));
        Ok(Self {
            query,
            _mapper_task: AbortOnDrop(mapper.abort_handle()),
            selection_lines,
            selection: None,
            target_model,
        })
    }
    pub async fn capture_selected(&mut self) -> Option<CapturedSelection> {
        let (spatial_ref, reparentable, reparent_lock, _) = self.selection.take()?;
        if let Err(_) = reparent_lock.lock().await {
            return None;
        }
        let root = self.selection_lines.client().get_root();
        let spatial = Spatial::create(root, Transform::none(), false).ok()?;
        spatial
            .set_relative_transform(
                &spatial_ref,
                Transform {
                    translation: Some([0.; 3].into()),
                    rotation: Some(Quat::IDENTITY.into()),
                    scale: None,
                },
            )
            .unwrap();
        _ = reparentable
            .parent(spatial.export_spatial().await.ok()?)
            .await;
        _ = self.selection_lines.set_lines(&[]);
        _ = self.target_model.set_enabled(true);
        {
            let bb = spatial_ref.get_local_bounding_box().await.ok()?;
            let longest = Vec3Component::find_longest(bb.size);
            let other_size = longest.other_max(bb.size);
            _ = self.target_model.set_spatial_parent(&spatial_ref);
            _ = self
                .target_model
                .set_local_transform(Transform::from_translation_rotation_scale(
                    bb.center,
                    longest.rotation() * Quat::from_rotation_y(f32::consts::FRAC_PI_2),
                    [other_size * 2.0; 3],
                ));
        }
        Some(CapturedSelection {
            spatial,
            reparentable,
            reparent_lock,
            target_model: self.target_model.clone(),
        })
    }
    pub async fn update_selection(&mut self, ray: Ray) {
        let mut closest_target = None;
        for obj @ (spatial, _, _, field) in self.query.iter().await.deref().values() {
            let distance = if let Some(field) = field {
                let Ok(raymarch_result) = field
                    .ray_march(&ray.ref_space, ray.origin, ray.direction)
                    .await
                else {
                    continue;
                };
                // field not hit
                if raymarch_result.min_distance > 0.0 {
                    continue;
                }
                raymarch_result.deepest_point_distance
            } else {
                let Ok(Some(pos)) = spatial
                    .get_transform(&ray.ref_space)
                    .await
                    .map(|t| t.translation)
                else {
                    continue;
                };
                let pos = Vec3::from(pos);
                let ray_relative = pos - ray.origin;
                let ray_distance = ray_relative.dot(ray.direction);
                // spatial is behind ray
                if ray_distance.is_sign_negative() {
                    continue;
                }
                let point_on_ray = ray.origin + (ray.direction * ray_distance);
                let distance_from_ray = pos.distance(point_on_ray);
                // a cone shape to make selecting far away objects easier
                if distance_from_ray > ray_distance * 0.1 {
                    continue;
                }

                distance_from_ray + ray_distance
            };
            if closest_target
                .as_ref()
                .is_none_or(|(dist, _)| distance < *dist)
            {
                closest_target.replace((distance, obj.clone()));
            }
        }
        let closest_target = closest_target.map(|v| v.1);
        self.selection = closest_target.clone();
        let Some(closest_target) = closest_target else {
            _ = self.selection_lines.set_lines(&[]);
            return;
        };
        _ = self.selection_lines.set_relative_transform(
            &closest_target.0,
            Transform {
                translation: Some(Vec3::ZERO.into()),
                rotation: Some(Quat::IDENTITY.into()),
                scale: None,
            },
        );
        let Ok(bb) = closest_target
            .0
            .get_relative_bounding_box(&self.selection_lines)
            .await
        else {
            warn!("can't get bounding box");
            _ = self.selection_lines.set_lines(&[]);
            return;
        };
        let mut lines = bounding_box(bb);
        lines
            .iter_mut()
            .for_each(|l| *l = l.clone().thickness(0.0025));
        _ = self.selection_lines.set_lines(&lines);
    }
}

#[derive(Debug, Clone)]
pub struct CapturedSelection {
    spatial: Spatial,
    target_model: Model,
    reparentable: ReparentableProxy<'static>,
    reparent_lock: ReparentLockProxy<'static>,
}

impl CapturedSelection {
    pub fn spatial(&self) -> &Spatial {
        &self.spatial
    }
}

impl Drop for CapturedSelection {
    fn drop(&mut self) {
        _ = self.target_model.set_enabled(false);
        _ = self
            .target_model
            .set_spatial_parent(self.spatial().client().get_root());
        tokio::task::block_in_place(|| {
            tokio::runtime::Handle::current().block_on(async {
                _ = self.reparentable.unparent().await;
                _ = self.reparent_lock.unlock().await;
            });
        });
    }
}

#[derive(Debug, Clone)]
pub struct Ray {
    pub origin: Vec3,
    pub direction: Vec3,
    pub ref_space: SpatialRef,
}

enum Vec3Component {
    X,
    Y,
    Z,
}
impl Vec3Component {
    fn find_longest(vec: impl Into<Vec3>) -> Self {
        let v = vec.into();
        if v.x >= v.y && v.x >= v.z {
            Self::X
        } else if v.y >= v.x && v.y >= v.z {
            Self::Y
        } else {
            Self::Z
        }
    }
    fn find_shortest(vec: impl Into<Vec3>) -> Self {
        let v = vec.into();
        if v.x <= v.y && v.x <= v.z {
            Self::X
        } else if v.y <= v.x && v.y <= v.z {
            Self::Y
        } else {
            Self::Z
        }
    }
    fn other_max(&self, vec: impl Into<Vec3>) -> f32 {
        let v = vec.into();
        match self {
            Vec3Component::X => v.y.max(v.z),
            Vec3Component::Y => v.x.max(v.z),
            Vec3Component::Z => v.x.max(v.y),
        }
    }
    fn rotation(&self) -> Quat {
        match self {
            Vec3Component::X => Quat::from_rotation_y(FRAC_PI_2) * Quat::from_rotation_x(FRAC_PI_2),
            Vec3Component::Y => Quat::IDENTITY,
            Vec3Component::Z => Quat::from_rotation_x(FRAC_PI_2),
        }
    }
    fn get(&self, vec: impl Into<Vec3>) -> f32 {
        let v = vec.into();
        match self {
            Vec3Component::X => v.x,
            Vec3Component::Y => v.y,
            Vec3Component::Z => v.z,
        }
    }
}

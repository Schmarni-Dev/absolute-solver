use std::{ops::Deref, sync::Arc};

use glam::{Quat, Vec3};
use stardust_xr_fusion::{
    ClientHandle,
    drawable::{Lines, LinesAspect},
    fields::{FieldRef, FieldRefAspect},
    list_query::{ListEvent, ObjectListQuery},
    node::NodeResult,
    objects::{
        interfaces::{ReparentLockProxy, ReparentableProxy},
        object_registry::ObjectRegistry,
    },
    query::ObjectQuery,
    spatial::{SpatialAspect, SpatialRef, SpatialRefAspect, Transform},
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
    _mapper_task: AbortOnDrop,
}

impl Selector {
    pub async fn new(
        client: Arc<ClientHandle>,
        object_registry: Arc<ObjectRegistry>,
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
        })
    }
    pub async fn find_selection(&self, capture_selection: bool, ray: Ray) {
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
        let Some(closest_target) = closest_target.map(|(_, v)| v) else {
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
pub struct Ray {
    pub origin: Vec3,
    pub direction: Vec3,
    pub ref_space: SpatialRef,
}

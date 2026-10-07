use bevy::asset::RenderAssetUsages;
use bevy::image::ImageSampler;
use bevy::prelude::*;
use bevy::render::render_resource::{Extent3d, TextureDimension, TextureFormat};
use stdr_core::OccupancyGrid;

use crate::sim::SimWorld;

/// The map image, rebuilt only when the engine's map revision moves.
#[derive(Resource, Default)]
pub struct MapTexture {
    pub image: Handle<Image>,
    /// The `map_revision` `image` was built from; 0 = no map yet.
    pub revision: u64,
}

#[derive(Component)]
pub struct MapSprite;

/// 0 → white, 100 → black, linear in between; unknown (−1) → grey (C++ `occupancy_to_rgba`).
fn gray(v: i8) -> u8 {
    if v < 0 {
        128
    } else {
        (255 - i32::from(v.min(100)) * 255 / 100) as u8
    }
}

/// Grid row 0 is the bottom, image row 0 the top: rows flip here, once.
pub fn grid_image(g: &OccupancyGrid) -> Image {
    let data = g
        .data()
        .chunks(g.width() as usize)
        .rev()
        .flatten()
        .flat_map(|&v| {
            let c = gray(v);
            [c, c, c, 255]
        })
        .collect();
    let mut image = Image::new(
        Extent3d {
            width: g.width(),
            height: g.height(),
            depth_or_array_layers: 1,
        },
        TextureDimension::D2,
        data,
        TextureFormat::Rgba8UnormSrgb,
        RenderAssetUsages::RENDER_WORLD,
    );
    // Crisp cells when zoomed in.
    image.sampler = ImageSampler::nearest();
    image
}

/// The world-space rectangle the map covers, in metres.
pub fn map_rect(g: &OccupancyGrid) -> Rect {
    let res = g.resolution() as f32;
    let min = Vec2::new(g.origin().x as f32, g.origin().y as f32);
    Rect::from_corners(
        min,
        min + Vec2::new(g.width() as f32, g.height() as f32) * res,
    )
}

pub fn sync_map_texture(
    sim: Res<SimWorld>,
    mut tex: ResMut<MapTexture>,
    mut images: ResMut<Assets<Image>>,
    sprite: Query<Entity, With<MapSprite>>,
    mut commands: Commands,
) {
    if tex.revision == sim.map_revision() {
        return;
    }
    let Some(grid) = sim.map() else { return };
    let image = images.add(grid_image(grid));
    *tex = MapTexture {
        image: image.clone(),
        revision: sim.map_revision(),
    };
    let rect = map_rect(grid);
    let bundle = (
        Sprite {
            image,
            custom_size: Some(rect.size()),
            ..default()
        },
        // Behind the gizmo overlay.
        Transform::from_translation(rect.center().extend(-1.0)),
        MapSprite,
    );
    match sprite.single() {
        Ok(e) => {
            commands.entity(e).insert(bundle);
        }
        Err(_) => {
            commands.spawn(bundle);
        }
    }
}

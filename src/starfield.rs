use bevy::prelude::*;
use bevy::render::render_resource::{
    Extent3d, TextureDimension, TextureFormat, TextureViewDescriptor, TextureViewDimension,
};
use rand::RngExt;

use crate::constants::*;

const FACE_SIZE: u32 = 512;

/// Generate the starfield cubemap and return an image handle.
pub fn create_starfield_cubemap(images: &mut Assets<Image>) -> Handle<Image> {
    let cubemap = generate_star_cubemap();
    images.add(cubemap)
}

fn generate_star_cubemap() -> Image {
    let mut rng = rand::rng();
    let face_pixels = FACE_SIZE * FACE_SIZE;
    let total_bytes = face_pixels as usize * 6 * 4;
    let mut data = vec![0u8; total_bytes];

    for face in 0..6 {
        let offset = face as usize * face_pixels as usize * 4;
        let face_data = &mut data[offset..offset + face_pixels as usize * 4];

        for _ in 0..STARFIELD_COUNT / 6 {
            let px = rng.random_range(0u32..FACE_SIZE);
            let py = rng.random_range(0u32..FACE_SIZE);
            let brightness: f32 = rng.random_range(0.4..1.0);

            // Slight color variation: warm white to cool white
            let temp: f32 = rng.random_range(-0.1..0.1);
            let r = ((brightness + temp).clamp(0.0, 1.0) * 255.0) as u8;
            let g = (brightness * 255.0) as u8;
            let b = ((brightness - temp).clamp(0.0, 1.0) * 255.0) as u8;

            let idx = ((py * FACE_SIZE + px) * 4) as usize;
            face_data[idx] = r;
            face_data[idx + 1] = g;
            face_data[idx + 2] = b;
            face_data[idx + 3] = 255;
        }
    }

    let mut image = Image::new(
        Extent3d {
            width: FACE_SIZE,
            height: FACE_SIZE,
            depth_or_array_layers: 6,
        },
        TextureDimension::D2,
        data,
        TextureFormat::Rgba8UnormSrgb,
        bevy::asset::RenderAssetUsages::RENDER_WORLD,
    );
    image.texture_view_descriptor = Some(TextureViewDescriptor {
        dimension: Some(TextureViewDimension::Cube),
        array_layer_count: Some(6),
        ..default()
    });
    image
}

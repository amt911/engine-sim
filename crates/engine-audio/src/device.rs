//! Audio device output through `cpal`. The callback only pops from a lock-free SPSC ring
//! buffer and writes samples: no allocation, no lock, no IO (Bencina's rules).
//! Excluded from coverage: needs a real sound card; `scripts/smoke.sh` covers it.

use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};
use cpal::{BufferSize, FromSample, SizedSample, StreamConfig};
use rtrb::{Consumer, Producer, RingBuffer};
use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};

#[derive(Debug, thiserror::Error)]
pub enum AudioError {
    #[error("no audio output device")]
    NoDevice,
    #[error("audio device error: {0}")]
    Device(String),
}

/// A running output stream. Keep it alive; dropping it stops the sound.
pub struct Output {
    _stream: cpal::Stream,
    pub sample_rate: u32,
    pub channels: u16,
    /// Frames the device found the ring empty for (an underrun: the simulation fell behind).
    pub dropouts: Arc<AtomicU64>,
}

/// Preferred device buffer, frames. Small for latency; ALSA defaults can be huge.
const DEVICE_FRAMES: u32 = 512;

/// Open the default output device. Returns the stream and the producer the simulation
/// writes mono samples into. `buffer_seconds` sizes the ring buffer.
pub fn open(buffer_seconds: f64) -> Result<(Output, Producer<f32>), AudioError> {
    let host = cpal::default_host();
    let device = host.default_output_device().ok_or(AudioError::NoDevice)?;
    let supported = device
        .default_output_config()
        .map_err(|e| AudioError::Device(e.to_string()))?;
    let format = supported.sample_format();
    let mut config: StreamConfig = supported.into();
    let sample_rate = config.sample_rate;
    let channels = config.channels;
    let cap = ((sample_rate as f64 * buffer_seconds) as usize).max(2048);
    let dropouts = Arc::new(AtomicU64::new(0));

    let try_build = |config: StreamConfig| -> Result<(cpal::Stream, Producer<f32>), AudioError> {
        let (producer, consumer) = RingBuffer::<f32>::new(cap);
        let stream = match format {
            cpal::SampleFormat::F32 => build::<f32>(&device, config, consumer, dropouts.clone()),
            cpal::SampleFormat::I16 => build::<i16>(&device, config, consumer, dropouts.clone()),
            cpal::SampleFormat::I32 => build::<i32>(&device, config, consumer, dropouts.clone()),
            cpal::SampleFormat::U16 => build::<u16>(&device, config, consumer, dropouts.clone()),
            other => {
                return Err(AudioError::Device(format!(
                    "unsupported sample format {other}"
                )));
            }
        }?;
        Ok((stream, producer))
    };

    config.buffer_size = BufferSize::Fixed(DEVICE_FRAMES);
    let (stream, producer) = match try_build(config) {
        Ok(ok) => ok,
        Err(_) => {
            config.buffer_size = BufferSize::Default;
            try_build(config)?
        }
    };
    stream
        .play()
        .map_err(|e| AudioError::Device(e.to_string()))?;
    Ok((
        Output {
            _stream: stream,
            sample_rate,
            channels,
            dropouts,
        },
        producer,
    ))
}

fn build<T>(
    device: &cpal::Device,
    config: StreamConfig,
    mut consumer: Consumer<f32>,
    dropouts: Arc<AtomicU64>,
) -> Result<cpal::Stream, AudioError>
where
    T: SizedSample + FromSample<f32>,
{
    let channels = config.channels as usize;
    let mut last = 0.0f32;
    device
        .build_output_stream(
            config,
            move |data: &mut [T], _: &cpal::OutputCallbackInfo| {
                for frame in data.chunks_mut(channels) {
                    // On underrun hold the last sample and let it decay, instead of zeros:
                    // zero-fill makes the device replay stale data ("the sound repeats").
                    last = match consumer.pop() {
                        Ok(s) => s,
                        Err(_) => {
                            // a relaxed atomic add: no lock, no allocation
                            dropouts.fetch_add(1, Ordering::Relaxed);
                            last * 0.995
                        }
                    };
                    let v = T::from_sample(last);
                    for s in frame.iter_mut() {
                        *s = v;
                    }
                }
            },
            |err| eprintln!("audio stream error: {err}"),
            None,
        )
        .map_err(|e| AudioError::Device(e.to_string()))
}

use burn::{
    backend::{wgpu::WgpuDevice, Wgpu},
    prelude::*,
    record::{BinBytesRecorder, FullPrecisionSettings, Recorder},
};

use shared::{Agent, AgentConfig};

use std::env;

static CONFIG_JSON: &str = include_str!("/tmp/rotten_mustard/config.json");
static MODEL_BYTES: &[u8] = include_bytes!("/tmp/rotten_mustard/model.bin");

fn main() {
    let args: Vec<String> = env::args().collect();

    let (pos_x, pos_z) = (
        args.get(1).expect("provide inputs"),
        args.get(2).expect("provide inputs"),
    );

    type MyBackend = Wgpu;
    let device = WgpuDevice::default();

    let config: AgentConfig =
        serde_json::from_str(CONFIG_JSON).expect("Failed to parse embedded config.json");

    let record = BinBytesRecorder::<FullPrecisionSettings>::default()
        .load(MODEL_BYTES.to_vec(), &device)
        .expect("Failed to load embedded model weights");

    let model: Agent<MyBackend> = config.init(&device).load_record(record);

    let mut combined_vec: Vec<f32> = Vec::with_capacity(2);

    match pos_x.parse::<f32>() {
        Ok(choice) => combined_vec.push(choice),
        Err(_) => println!("Invalid x"),
    }

    match pos_z.parse::<f32>() {
        Ok(choice) => combined_vec.push(choice),
        Err(_) => println!("Invalid z"),
    }

    if combined_vec.len() == 2 {
        let choices_tensor =
            Tensor::<MyBackend, 1>::from_floats(combined_vec.as_slice(), &device).reshape([1, 2]);

        let choices = model.forward(choices_tensor);

        let choices_vec: Vec<f32> = choices.to_data().to_vec().unwrap();
        println!("Choice: {:?}", choices_vec);
    }
}

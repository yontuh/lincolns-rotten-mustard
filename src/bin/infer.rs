use ipc_channel::ipc::{self, IpcOneShotServer};
use shared::{Handshake, ModelChoices, Rewards};
use std::process::Command;

use std::io::{self, Write};

use burn::{
    backend::{wgpu::WgpuDevice, Wgpu},
    prelude::*,
    record::{CompactRecorder, Recorder},
};

use shared::{Agent, AgentConfig};

use std::thread;
use std::time::Duration;

fn main() {
    type MyBackend = Wgpu;
    let device = WgpuDevice::default();
    let artifact_dir = "/tmp/rotten_mustard";

    // Load Config and Model
    let config =
        AgentConfig::load(format!("{}/config.json", artifact_dir)).expect("Run training first");
    let record = CompactRecorder::new()
        .load(format!("{}/model", artifact_dir).into(), &device)
        .expect("Run training first");
    let model: Agent<MyBackend> = config.init(&device).load_record(record);

    let (server, server_name) = IpcOneShotServer::new().unwrap();
    println!("[Model] IPC Server started. Spawning simulation...");

    let mut child = Command::new("cargo")
        .args(["run", "--quiet", "--bin", "physics", "--", &server_name])
        .spawn()
        .expect("Failed to spawn simulation");

    let (_, handshake): (_, Handshake) = server.accept().unwrap();
    let tx_choice = handshake.tx_choice;

    let (tx_reward, rx_reward) = ipc::channel::<Rewards>().unwrap();

    handshake.tx_back_channel.send(tx_reward).unwrap();

    println!("[Model] Handshake complete.");

    thread::sleep(Duration::from_millis(200));

    loop {
        print!("\nx value:");
        io::stdout().flush().unwrap();
        let mut buffer = String::new();
        if io::stdin().read_line(&mut buffer).is_err() {
            break;
        }
        let x_choice = buffer.trim();

        print!("\nz value:");
        io::stdout().flush().unwrap();
        let mut buffer = String::new();
        if io::stdin().read_line(&mut buffer).is_err() {
            break;
        }
        let z_choice = buffer.trim();

        let mut model_choices = ModelChoices {
            yaws: Vec::with_capacity(1),
            powers: Vec::with_capacity(1),
            x_vec: Vec::with_capacity(1),
            z_vec: Vec::with_capacity(1),
        };

        let mut combined_vec: Vec<f32> = Vec::with_capacity(2);

        match x_choice.parse::<f32>() {
            Ok(choice) => {
                model_choices.x_vec.push(choice);
                combined_vec.push(choice);
            }
            Err(_) => {
                println!("Invalid number");
                let _ = child.wait();
            }
        }

        match z_choice.parse::<f32>() {
            Ok(choice) => {
                model_choices.z_vec.push(choice);
                combined_vec.push(choice);
            }
            Err(_) => {
                println!("Invalid number");
                let _ = child.wait();
            }
        }

        let choices_tensor =
            Tensor::<MyBackend, 1>::from_floats(combined_vec.as_slice(), &device).reshape([1, 2]);

        let choices = model.forward(choices_tensor);

        let choices_vec: Vec<f32> = choices.to_data().to_vec().unwrap();

        println!("Choice: {:?}", choices_vec);

        model_choices.yaws.push(choices_vec[0]);

        model_choices.powers.push(choices_vec[1]);

        tx_choice.send(model_choices).unwrap();

        let rewards = rx_reward.recv().unwrap();

        println!("Rewards: {:?}", rewards);
    }
    let _ = child.wait();
}

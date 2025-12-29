use ipc_channel::ipc::{self, IpcOneShotServer};
use rand::prelude::*;
use shared::{Handshake, ModelChoice, ModelChoices, Pos, Poses, Reward};
use std::process::Command;

use burn::{
    backend::{wgpu::WgpuDevice, Autodiff, Wgpu},
    optim::{AdamConfig, GradientsParams, Optimizer},
    prelude::*,
    record::CompactRecorder,
};
use rand_distr::{Distribution, Normal};
use shared::{Agent, AgentConfig};

const ARTIFACT_DIR: &str = "/tmp/burn_artillery";
const NUM_ITERATIONS: usize = 5000; // Number of batch updates
const BATCH_SIZE: usize = 128; // Larger values generalize the gradient, finds the average value
                               // on a larger (and therefore more predictable) plane
const LEARNING_RATE: f64 = 0.001; // How aggressively to backpropagate
const HIDDEN_SIZE: usize = 64; // We first set it here

fn main() {
    let (server, server_name) = IpcOneShotServer::new().unwrap();
    println!("[Model] IPC Server started. Spawning simulation...");

    let mut child = Command::new("cargo")
        .args(["run", "--quiet", "--bin", "physics", "--", &server_name])
        .spawn()
        .expect("Failed to spawn simulation");

    let (_, handshake): (_, Handshake) = server.accept().unwrap();
    let tx_choice = handshake.tx_choice;

    let (tx_reward, rx_reward) = ipc::channel::<Reward>().unwrap();

    handshake.tx_back_channel.send(tx_reward).unwrap();

    println!("[Model] Handshake complete. Starting loop.");

    for _i in 0..500 {
        let choice = ModelChoice {
            yaw: 45.0,
            power: 8.5,
        };
        println!("\n[Model] Sending Choice: {:?}", choice);
        tx_choice.send(choice).unwrap();

        let reward = rx_reward.recv().unwrap();
        println!("[Model] Received Reward: {:?}", reward);

        // thread::sleep(Duration::from_millis(1000));
    }

    let _ = child.wait();
}

// Quantity assumes 0 indexing!
fn generate_choices(quantity: i32, poses: Poses) -> ModelChoices {
    let mut model_choices: ModelChoices = ModelChoices {
        yaws: Vec::with_capacity(quantity as usize),
        powers: Vec::with_capacity(quantity as usize),
    };

    let mut rng = rand::rng();

    return model_choices;
}

// Quantity assumes 0 indexing!
fn generate_poses(quantity: i32) -> Poses {
    let mut poses = Poses {
        x_vec: Vec::with_capacity(quantity as usize),
        z_vec: Vec::with_capacity(quantity as usize),
    };

    let mut rng = rand::rng();
    loop {
        // Feet!
        let x_feet = rng.random_range(-5.1..5.1);
        let z_feet = rng.random_range(-5.1..5.1);

        if !(x_feet > 3.0 && z_feet > 3.0) {
            poses.x_vec.push(x_feet);
            poses.z_vec.push(z_feet);
        }
        if poses.x_vec.len() == quantity as usize {
            break;
        }
    }
    return poses;
}

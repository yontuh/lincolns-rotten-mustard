use ipc_channel::ipc::{self, IpcOneShotServer};
use rand::prelude::*;
use shared::{Handshake, ModelChoice, ModelChoices, Pos, Poses, Reward, Rewards};
use std::process::Command;

use burn::{
    backend::{wgpu::WgpuDevice, Autodiff, Wgpu},
    optim::{AdamConfig, GradientsParams, Optimizer},
    prelude::*,
    record::CompactRecorder,
};
use rand_distr::{Distribution, Normal};
use shared::{Agent, AgentConfig};

use std::fs::File;
use std::io::Write;

const ARTIFACT_DIR: &str = "/tmp/rotten_mustard";
const NUM_ITERATIONS: usize = 5000; // Number of batch updates
const BATCH_SIZE: usize = 10; // Larger values generalize the gradient, finds the average value
                              // on a larger (and therefore more predictable) plane
const LEARNING_RATE: f64 = 0.001; // How aggressively to backpropagate
const HIDDEN_SIZE: usize = 64; // We first set it here

fn main() {
    let (server, server_name) = IpcOneShotServer::new().unwrap();
    println!("[Model] IPC Server started. Spawning simulation...");

    let mut child = Command::new("cargo")
        .args([
            "run",
            "--quiet",
            "--release",
            "--bin",
            "physics",
            "--",
            &server_name,
        ])
        .spawn()
        .expect("Failed to spawn simulation");

    // ---

    let mut file = File::create("the_stats.give_a_follow").unwrap();

    // ---

    let (_, handshake): (_, Handshake) = server.accept().unwrap();
    let tx_choice = handshake.tx_choice;

    let (tx_reward, rx_reward) = ipc::channel::<Rewards>().unwrap();

    handshake.tx_back_channel.send(tx_reward).unwrap();

    println!("[Model] Handshake complete.");

    // -------------------------------------------------

    type MyBackend = Autodiff<Wgpu>;
    let device = WgpuDevice::default();

    let mut model: Agent<MyBackend> = AgentConfig::new(HIDDEN_SIZE).init(&device);
    // let mut optimizer = AdamConfig::new().init();

    for i in 0..NUM_ITERATIONS {
        let poses = generate_poses(BATCH_SIZE);
        let mut model_choices: ModelChoices = ModelChoices {
            yaws: Vec::with_capacity(BATCH_SIZE),
            powers: Vec::with_capacity(BATCH_SIZE),
            x_vec: poses.x_vec,
            z_vec: poses.z_vec,
        };
        println!("Model Choices: {:?}", model_choices.x_vec);

        let mut combined_inputs: Vec<f32> = Vec::with_capacity(BATCH_SIZE * 2);
        for i in 0..BATCH_SIZE {
            combined_inputs.push(model_choices.x_vec[i]);
            combined_inputs.push(model_choices.z_vec[i]);
        }

        let inputs_tensor =
            Tensor::<MyBackend, 1>::from_floats(combined_inputs.as_slice(), &device)
                .reshape([BATCH_SIZE, 2]);

        let mus = model.forward(inputs_tensor);

        let mus_reshaped = mus.reshape([BATCH_SIZE * 2, 1]);

        let mus_vec: Vec<f32> = mus_reshaped.to_data().to_vec().unwrap();

        for i in 0..BATCH_SIZE {
            model_choices.yaws.push(combined_inputs[i]);
            model_choices.powers.push(combined_inputs[i + 1]);
        }

        // Currently nothing done with Gaussian whatever!

        let mut rng = rand::rng();

        println!(
            "\n[Model] Sending Choices. Example, yaw: {:?}, power: {:?}",
            model_choices.yaws[0], model_choices.powers[0]
        );
        println!("2nd Model Choices: {:?}", model_choices.x_vec);
        tx_choice.send(model_choices).unwrap();

        let rewards = rx_reward.recv().unwrap();
        println!("[Model] Received Rewards: {:?}", rewards);

        println!("🎃🎃🎃🎃 Iteration: {} 🎃🎃🎃🎃", i);

        writeln!(file, "🎃🎃🎃🎃 Iteration: {} 🎃🎃🎃🎃", i).unwrap();

        writeln!(file, "{:?}", rewards.rewards).unwrap();

        // thread::sleep(Duration::from_millis(1000));
    }

    let _ = child.wait();
}

// Quantity assumes 1 indexing!
fn generate_poses(quantity: usize) -> Poses {
    let mut poses = Poses {
        x_vec: Vec::with_capacity(quantity),
        z_vec: Vec::with_capacity(quantity),
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
        if poses.x_vec.len() == quantity {
            println!("generate poses x values: {:?}", poses.x_vec);
            break;
        }
    }
    return poses;
}

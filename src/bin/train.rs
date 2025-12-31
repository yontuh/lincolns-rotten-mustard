use ipc_channel::ipc::{self, IpcOneShotServer};
use rand::prelude::*;
use shared::{Handshake, ModelChoices, Poses, Rewards};
use std::process::Command;

use burn::{
    backend::{wgpu::WgpuDevice, Autodiff, Wgpu},
    optim::{AdamConfig, GradientsParams, Optimizer},
    prelude::*,
    record::{BinFileRecorder, FullPrecisionSettings},
};
use rand_distr::{Distribution, Normal};
use shared::{Agent, AgentConfig};

use std::fs::File;
use std::io::Write;

const ARTIFACT_DIR: &str = "/tmp/rotten_mustard";
const NUM_ITERATIONS: usize = 5000; // Number of batch updates
const BATCH_SIZE: usize = 64; // Larger values generalize the gradient, finds the average value
                              // on a larger (and therefore more predictable) plane
const LEARNING_RATE: f64 = 0.001; // How aggressively to backpropagate
const HIDDEN_SIZE: usize = 64; // We first set it here

fn main() {
    let (server, server_name) = IpcOneShotServer::new().unwrap();
    println!("[Model] IPC Server started. Spawning simulation...");

    let mut physics_bin = Command::new("cargo")
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

    let (_, handshake): (_, Handshake) = server.accept().unwrap();
    let tx_choice = handshake.tx_choice;

    let (tx_reward, rx_reward) = ipc::channel::<Rewards>().unwrap();

    handshake.tx_back_channel.send(tx_reward).unwrap();

    println!("[Model] Handshake complete.");

    // -------------------------------------------------

    let mut file = File::create("the_stats.give_a_follow").unwrap();

    // -------------------------------------------------

    type MyBackend = Autodiff<Wgpu>;
    let device = WgpuDevice::default();

    let mut model: Agent<MyBackend> = AgentConfig::new(HIDDEN_SIZE).init(&device);
    let mut optimizer = AdamConfig::new().init();

    let yaw_normal_dist = Normal::new(0.0, 2.0).unwrap();
    let power_normal_dist = Normal::new(0.0, 0.2).unwrap();
    let mut rng = rand::rng();

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

        let mus_reshaped = mus.clone().reshape([BATCH_SIZE * 2, 1]);

        let mus_vec: Vec<f32> = mus_reshaped.to_data().to_vec().unwrap();

        let mut mus_vec_deviated: Vec<f32> = Vec::with_capacity(mus_vec.len() * 2);

        // Add Gaussian deviation
        for i in 0..BATCH_SIZE {
            let offset = i * 2;
            let yaw_noise = yaw_normal_dist.sample(&mut rng);
            let power_noise = power_normal_dist.sample(&mut rng);
            model_choices.yaws.push(mus_vec[offset] + yaw_noise);
            model_choices.powers.push(mus_vec[offset + 1] + power_noise);
            mus_vec_deviated.push(mus_vec[offset] + yaw_noise);
            mus_vec_deviated.push(mus_vec[offset + 1] + power_noise);
        }

        println!(
            "\n[Model] Sending Choices. Example, yaw: {:?}, power: {:?}",
            model_choices.yaws[0], model_choices.powers[0]
        );
        tx_choice.send(model_choices).unwrap();

        let rewards = rx_reward.recv().unwrap();

        let rewards_tensor =
            Tensor::<MyBackend, 1>::from_floats(rewards.rewards.as_slice(), &device)
                .reshape([BATCH_SIZE, 1]);

        let actions_tensor =
            Tensor::<MyBackend, 1>::from_floats(mus_vec_deviated.as_slice(), &device)
                .reshape([BATCH_SIZE, 2]);

        // The Math: (mu - action)^2 * reward
        // If reward is 0, loss is 0.
        // If reward is 1, loss is MSE(mu, action).
        // Detach actions and rewards so we don't backprop through them.
        // .detach() removes actions_tensor from Autodiff graph
        // The subtraction would ordinarily link the actions_tensor to being part of the Autodiff Graph
        // Gives a tensor with how
        // [our predictions] - [our deviated predictions]
        let diff = mus - actions_tensor.detach();
        // Raise diff to a power. 'f' stands for float
        let squared_error = diff.powf_scalar(2.0);
        // Total average (mean) loss of this batch
        let loss = (squared_error * rewards_tensor.detach()).mean();

        // 5. Optimize
        // Backward pass to get gradients
        let grads = loss.backward();
        // No understanding of what is happening here.
        // Must not be important enough to be included as a step in the optimizer?
        let grads_params = GradientsParams::from_grads(grads, &model);
        // Apply gradient descent and update the model
        model = optimizer.step(LEARNING_RATE, model, grads_params);

        println!("🎃🎃🎃🎃 Iteration: {} 🎃🎃🎃🎃", i);

        writeln!(file, "🎃🎃🎃🎃 Iteration: {} 🎃🎃🎃🎃", i).unwrap();

        writeln!(file, "{:?}", rewards.rewards).unwrap();
    }

    std::fs::create_dir_all(ARTIFACT_DIR).ok();
    AgentConfig {
        hidden_size: HIDDEN_SIZE,
    }
    .save(format!("{}/config.json", ARTIFACT_DIR))
    .unwrap();
    model
        .save_file(
            format!("{}/model", ARTIFACT_DIR),
            // Can also do HalfPrecisionSettings and DoublePrecisionSettings
            &BinFileRecorder::<FullPrecisionSettings>::new(),
        )
        .unwrap();

    println!("Training Done");

    let _ = physics_bin.wait();
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

        if !(x_feet > 4.3 && z_feet > 4.3) {
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

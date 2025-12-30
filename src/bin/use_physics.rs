use ipc_channel::ipc::{self, IpcOneShotServer};
use shared::{Handshake, ModelChoice, ModelChoices, Pos, Poses, Reward, Rewards};
use std::process::Command;

use std::io::{self, Write};

use rand_distr::{Distribution, Normal};
use shared::{Agent, AgentConfig};

use std::fs::File;

use std::thread;
use std::time::Duration;

fn main() {
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
        print!("\nyaw:");
        io::stdout().flush().unwrap();

        let mut buffer1 = String::new();
        if io::stdin().read_line(&mut buffer1).is_err() {
            break;
        }
        let yaw_choice = buffer1.trim();
        // ---

        print!("\npower:");
        io::stdout().flush().unwrap();
        let mut buffer = String::new();
        if io::stdin().read_line(&mut buffer).is_err() {
            break;
        }
        let power_choice = buffer.trim();

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

        match z_choice.parse::<f32>() {
            Ok(choice) => {
                model_choices.z_vec.push(choice);
            }
            Err(_) => println!("Invalid number"),
        }

        match x_choice.parse::<f32>() {
            Ok(choice) => {
                model_choices.x_vec.push(choice);
            }
            Err(_) => println!("Invalid number"),
        }

        match yaw_choice.parse::<f32>() {
            Ok(choice) => {
                model_choices.yaws.push(choice);
            }
            Err(_) => println!("Invalid number"),
        }

        match power_choice.parse::<f32>() {
            Ok(choice) => {
                model_choices.powers.push(choice);
            }
            Err(_) => println!("Invalid number"),
        }

        tx_choice.send(model_choices).unwrap();

        let rewards = rx_reward.recv().unwrap();

        println!("Rewards: {:?}", rewards);
    }
}

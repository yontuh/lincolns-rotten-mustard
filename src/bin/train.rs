use ipc_channel::ipc::{self, IpcOneShotServer};
use shared::{Handshake, ModelChoice, Reward};
use std::process::Command;

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

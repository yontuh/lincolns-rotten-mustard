use bevy::app::ScheduleRunnerPlugin;
use bevy::input::mouse::AccumulatedMouseMotion;
use bevy::prelude::*;
use bevy_rapier3d::prelude::*;
use ipc_channel::ipc::{self, IpcSender};
use iyes_perf_ui::prelude::*;
use shared::{Handshake, ModelChoices, Rewards};
use std::env;
use std::sync::Mutex;
use std::time::Duration;
use std::{
    f32::consts::{FRAC_PI_2, PI},
    ops::Range,
};

const NUM_ARENAS: usize = 64;

const ARENA_SPACING: f32 = 50.0;

fn main() {
    let headless = false;

    let mut app = App::new();

    if headless {
        app.add_plugins(
            MinimalPlugins.set(ScheduleRunnerPlugin::run_loop(Duration::from_secs_f64(0.0))),
        );
    } else {
        app.add_plugins(DefaultPlugins)
            .add_plugins(bevy::diagnostic::FrameTimeDiagnosticsPlugin::default())
            .add_plugins(bevy::diagnostic::EntityCountDiagnosticsPlugin)
            .add_plugins(bevy::diagnostic::SystemInformationDiagnosticsPlugin)
            .add_plugins(bevy::render::diagnostic::RenderDiagnosticsPlugin)
            .add_plugins(PerfUiPlugin)
            .add_plugins(RapierDebugRenderPlugin::default())
            .add_systems(Startup, setup_graphics)
            .add_systems(FixedUpdate, move_robot)
            .add_systems(FixedUpdate, orbit);
    }

    app.init_resource::<CameraSettings>()
        .init_resource::<CameraState>()
        .add_plugins(RapierPhysicsPlugin::<NoUserData>::default())
        .add_systems(Startup, setup_physics_speed)
        .add_systems(Startup, setup_physics)
        .add_systems(Startup, setup_ipc)
        .add_systems(FixedUpdate, run_training_loop)
        .run();
}

fn setup_physics_speed(mut timestep_mode: ResMut<TimestepMode>) {
    *timestep_mode = TimestepMode::Fixed {
        dt: 1.0 / 60.0,
        substeps: 1,
    };
}

fn setup_ipc(mut commands: Commands) {
    let args: Vec<String> = env::args().collect();
    let server_name = args.get(1).expect("provide name");

    let (tx_choice, rx_choice) = ipc::channel::<ModelChoices>().unwrap();
    let (tx_back_channel, rx_back_channel) = ipc::channel::<IpcSender<Rewards>>().unwrap();

    let tx_bootstrap: IpcSender<Handshake> = IpcSender::connect(server_name.to_string()).unwrap();

    tx_bootstrap
        .send(Handshake {
            tx_choice,
            tx_back_channel,
        })
        .unwrap();

    let tx_reward = rx_back_channel.recv().unwrap();

    commands.insert_resource(TrainingConnection {
        rx_choices: Mutex::new(rx_choice),
        tx_reward: Mutex::new(tx_reward),
        waiting_for_result: false,
        pending_setup: false,
        pending_launch: false,
        index: 0,
        pending_yaws: Vec::new(),
        pending_powers: Vec::new(),
        pending_x_pos: Vec::new(),
        pending_z_pos: Vec::new(),
        shot_frame_counter: 0,
        collected_rewards: Vec::new(),
        finished_count: 0,
        batch_size: None,
    });
    println!("Done with IPC setup");
}

fn run_training_loop(
    mut commands: Commands,
    mut connection: ResMut<TrainingConnection>,
    mut take_out_query: Query<(&mut Transform, &mut RobotObject, &ArenaIndex), Without<Ball>>,
    mut collision_events: EventReader<CollisionEvent>,
    reset_query: Query<Entity, With<ShouldReset>>,
    goal_query: Query<&GoalObject>,
    ball_query: Query<(Entity, &ArenaIndex), With<Ball>>,
    missed_query: Query<(Entity, &Transform, &ArenaIndex), With<Ball>>,
) {
    if connection.pending_launch {
        take_out(
            &mut commands,
            &mut take_out_query,
            &connection.pending_yaws,
            &connection.pending_powers,
        );
        connection.pending_launch = false;
        connection.waiting_for_result = true;
    }

    // Setup the Arena
    if connection.pending_setup {
        reset_with_pos(
            &mut commands,
            &reset_query,
            &connection.pending_x_pos,
            &connection.pending_z_pos,
        );
        println!(
            "[sim] Batch setup complete for {} agents",
            connection.batch_size.unwrap()
        );
        connection.pending_setup = false;
        connection.pending_launch = true;
        return;
    }

    // Wait for and parse connection
    if !connection.waiting_for_result {
        // Halts until connection
        let received = connection.rx_choices.lock().unwrap().try_recv();

        match received {
            Ok(choices) => {
                println!(
                    "[sim] Received {:?} choices. Example yaws: {:?}, power: {:?} ",
                    choices.yaws.len(),
                    choices.yaws[0],
                    choices.powers[0]
                );

                // println!("Recieved Choices: {:?}", choices.x_vec);
                let batch_size = choices.yaws.len();

                connection.pending_yaws = choices.yaws;
                connection.pending_powers = choices.powers;
                connection.pending_x_pos = choices.x_vec;
                connection.pending_z_pos = choices.z_vec;

                // Rewards is reset in the else clause

                // println!("Pending_x_poses: {:?}", connection.pending_x_pos);

                // [None, None, None, ...]
                connection.collected_rewards = vec![None; batch_size];
                connection.finished_count = 0;

                connection.pending_setup = true;
                connection.batch_size = Some(batch_size);

                connection.shot_frame_counter = 0;

                connection.index = 0;
            }
            Err(_) => {}
        }
    // Track simulation
    } else {
        connection.shot_frame_counter += 1;

        let successes = check_success(
            &mut commands,
            &mut collision_events,
            &goal_query,
            &ball_query,
        );
        for (idx, reward) in successes {
            if idx < connection.collected_rewards.len()
                && connection.collected_rewards[idx].is_none()
            {
                connection.collected_rewards[idx] = Some(reward);
                connection.finished_count += 1;
            }
        }

        let misses = check_missed(&mut commands, &missed_query);
        for (idx, reward) in misses {
            if idx < connection.collected_rewards.len()
                && connection.collected_rewards[idx].is_none()
            {
                connection.collected_rewards[idx] = Some(reward);
                connection.finished_count += 1;
            }
        }

        if let Some(batch_size) = connection.batch_size {
            if connection.finished_count >= batch_size {
                let final_rewards: Vec<f32> = connection
                    // Vec<Option<f32>>
                    .collected_rewards
                    .iter()
                    .map(|r| r.unwrap_or(0.0))
                    .collect();

                println!("[sim] Sending {} Rewards", final_rewards.len());

                connection
                    .tx_reward
                    .lock()
                    .unwrap()
                    .send(Rewards {
                        rewards: final_rewards,
                    })
                    .unwrap();

                connection.waiting_for_result = false;
            }
        }
    }
}

const FTM: f32 = 0.3048;

const POUNDS_TO_KILOGRAMS: f32 = 0.453592;

const WALL_HEIGHT: f32 = 1.0 * FTM;

const WALL_LENGTH: f32 = 12.0 * FTM;

const ROBOT_LENGTH: f32 = (13.858 / 12.0) * FTM;

const ROBOT_HEIGHT: f32 = (6.0 / 12.0) * FTM;

const ROBOT_WEIGHT: f32 = 20.0 * POUNDS_TO_KILOGRAMS;

const BALL_RAD: f32 = ((5.0 / 12.0) / 2.0) * FTM;

const BALL_WEIGHT: f32 = 0.165 * POUNDS_TO_KILOGRAMS;

const GOAL_HEIGHT: f32 = 0.9845;

fn spawn_arena_objects(commands: &mut Commands, offset: Vec3) {
    let transform_input = Vec3::new(0.0, -0.001, 0.0) + offset;
    commands.spawn((
        Collider::cuboid(WALL_LENGTH / 2.0, 0.001, WALL_LENGTH / 2.0),
        Transform::from_xyz(transform_input.x, transform_input.y, transform_input.z),
        PhysicsObject,
    ));

    let transform_input = Vec3::new(WALL_LENGTH / 2.0, WALL_HEIGHT / 2.0, 0.0) + offset;

    commands.spawn((
        Collider::cuboid(0.001, WALL_HEIGHT / 2.0, WALL_LENGTH / 2.0),
        Transform::from_xyz(transform_input.x, transform_input.y, transform_input.z),
        PhysicsObject,
    ));

    let transform_input = Vec3::new(-WALL_LENGTH / 2.0, WALL_HEIGHT / 2.0, 0.0) + offset;
    commands.spawn((
        Collider::cuboid(0.001, WALL_HEIGHT / 2.0, WALL_LENGTH / 2.0),
        Transform::from_xyz(transform_input.x, transform_input.y, transform_input.z),
        PhysicsObject,
    ));

    let transform_input = Vec3::new(0.0, WALL_HEIGHT / 2.0, WALL_LENGTH / 2.0) + offset;
    commands.spawn((
        Collider::cuboid(WALL_LENGTH / 2.0, WALL_HEIGHT / 2.0, 0.001),
        Transform::from_xyz(transform_input.x, transform_input.y, transform_input.z),
        PhysicsObject,
    ));

    let transform_input = Vec3::new(0.0, WALL_HEIGHT / 2.0, -WALL_LENGTH / 2.0) + offset;

    commands.spawn((
        Collider::cuboid(WALL_LENGTH / 2.0, WALL_HEIGHT / 2.0, 0.001),
        Transform::from_xyz(transform_input.x, transform_input.y, transform_input.z),
        PhysicsObject,
    ));

    let transform_input = Vec3::new(1.479, GOAL_HEIGHT / 2.0, 1.479) + offset;

    commands.spawn((
        Collider::cuboid(0.495, GOAL_HEIGHT / 2.0, 0.001),
        Transform::from_xyz(transform_input.x, transform_input.y, transform_input.z)
            .with_rotation(Quat::from_euler(EulerRot::YXZ, PI / 4.0, 0.0, 0.0)),
        PhysicsObject,
    ));
    let transform_input = Vec3::new(
        (WALL_LENGTH / 2.0) - 0.35,
        GOAL_HEIGHT / 2.0,
        WALL_LENGTH / 2.0,
    ) + offset;
    commands.spawn((
        Collider::cuboid(0.35, GOAL_HEIGHT / 2.0, 0.001),
        Transform::from_xyz(transform_input.x, transform_input.y, transform_input.z),
        PhysicsObject,
    ));

    let transform_input =
        Vec3::new((WALL_LENGTH / 2.0) - 0.7, GOAL_HEIGHT, WALL_LENGTH / 2.0) + offset;

    commands.spawn((
        Collider::triangle(
            Vec3::new(0.0, 0.0, 0.0) + offset,
            Vec3::new(0.70, 0.0, 0.0) + offset,
            Vec3::new(0.70, 0.375, 0.0) + offset,
        ),
        Transform::from_xyz(transform_input.x, transform_input.y, transform_input.z),
        PhysicsObject,
    ));

    let transform_input = Vec3::new(
        WALL_LENGTH / 2.0,
        GOAL_HEIGHT / 2.0,
        (WALL_LENGTH / 2.0) - 0.35,
    ) + offset;

    commands.spawn((
        Collider::cuboid(0.001, GOAL_HEIGHT / 2.0, 0.35),
        Transform::from_xyz(transform_input.x, transform_input.y, transform_input.z),
        PhysicsObject,
    ));

    let transform_input =
        Vec3::new(WALL_LENGTH / 2.0, GOAL_HEIGHT, (WALL_LENGTH / 2.0) - 0.7) + offset;

    commands.spawn((
        Collider::triangle(
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 0.70),
            Vec3::new(0.0, 0.375, 0.70),
        ),
        Transform::from_xyz(transform_input.x, transform_input.y, transform_input.z),
        PhysicsObject,
    ));

    let transform_input = Vec3::new(
        WALL_LENGTH / 2.0,
        GOAL_HEIGHT - 0.2,
        (WALL_LENGTH / 2.0) - 0.7,
    ) + offset;

    commands.spawn((
        Sensor,
        ActiveEvents::COLLISION_EVENTS,
        Collider::triangle(
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 0.7),
            Vec3::new(-0.7, 0.0, 0.7),
        ),
        Transform::from_xyz(transform_input.x, transform_input.y, transform_input.z),
        GoalObject,
    ));
}

fn spawn_robot_objects(pos: Transform, arena: usize, commands: &mut Commands) {
    // Robot
    commands
        .spawn((
            Velocity::default(),
            ExternalImpulse::default(),
            Ccd::enabled(),
            LockedAxes::ROTATION_LOCKED_X | LockedAxes::ROTATION_LOCKED_Z,
            RigidBody::Dynamic,
            Collider::cuboid(ROBOT_LENGTH / 2.0, ROBOT_HEIGHT / 2.0, ROBOT_LENGTH / 2.0),
            pos,
            ColliderMassProperties::Mass(ROBOT_WEIGHT),
            ReadMassProperties::default(),
            Friction::coefficient(0.0),
            Damping {
                linear_damping: 0.0,
                angular_damping: 0.0,
            },
            RobotObject {
                linear_speed: 10.50,
                turn_speed: 3.5,
                max_impulse: 0.75, // "Torque"
                snozzle_angle: 80.0,
                snozzle_pow: 6.5, // m/s,
                stiffness: 0.5,
            },
            ArenaIndex(arena),
            CollisionGroups::new(Group::GROUP_1, Group::ALL ^ Group::GROUP_2),
        ))
        .insert((PhysicsObject, DebugObject, ShouldReset));
}

fn take_out(
    commands: &mut Commands,
    query: &mut Query<(&mut Transform, &mut RobotObject, &ArenaIndex), Without<Ball>>,
    yaws: &Vec<f32>,
    powers: &Vec<f32>,
) {
    for (mut transform, mut robot, arena) in query {
        let idx = arena.0;
        if idx >= yaws.len() {
            continue;
        }

        let yaw = yaws[idx];
        let power = powers[idx];

        let adjusted_power = power + 7.0;

        robot.snozzle_pow = adjusted_power;

        let adjusted_yaw = yaw + 45.0;

        transform.rotation = Quat::from_rotation_y(adjusted_yaw.to_radians());

        let pitch_radians = -robot.snozzle_angle.to_radians();

        let pitch_rotation = Quat::from_rotation_x(pitch_radians);

        let final_rotation = transform.rotation * pitch_rotation;

        let forward_dir = final_rotation * Vec3::Z;
        let shoot_velocity = forward_dir * robot.snozzle_pow;

        let spawn_pos = transform.translation + (Vec3::Y * ROBOT_HEIGHT);

        commands.spawn(BallBundle::new(spawn_pos, shoot_velocity, arena.0));
    }
}

fn check_success(
    commands: &mut Commands,
    collision_events: &mut EventReader<CollisionEvent>,
    goal_query: &Query<&GoalObject>,
    ball_query: &Query<(Entity, &ArenaIndex), With<Ball>>,
) -> Vec<(usize, f32)> {
    let mut results = Vec::new();

    for event in collision_events.read() {
        if let CollisionEvent::Started(e1, e2, _) = event {
            let ball_info = if goal_query.contains(*e1) {
                ball_query.get(*e2).ok()
            } else if goal_query.contains(*e2) {
                ball_query.get(*e1).ok()
            } else {
                None
            };

            if let Some((entity, arena)) = ball_info {
                commands.entity(entity).despawn();
                results.push((arena.0, 1.0));
                println!("✅ goaled ✅✅✅✅✅✅✅");
            }
        }
    }
    results
}

fn check_missed(
    commands: &mut Commands,
    query: &Query<(Entity, &Transform, &ArenaIndex), With<Ball>>,
) -> Vec<(usize, f32)> {
    let mut results = Vec::new();
    for (entity, transform, arena) in query {
        if transform.translation.y <= BALL_RAD + 0.05 {
            println!("♈♈ missed ♈♈");
            commands.entity(entity).despawn();
            results.push((arena.0, 0.0));
        }
    }
    results
}

fn move_robot(
    keys: Res<ButtonInput<KeyCode>>,
    mut query: Query<(
        &mut ExternalImpulse,
        &Velocity,
        &Transform,
        &RobotObject,
        &ReadMassProperties,
    )>,
) {
    for (mut impulse, velocity, transform, robot, mass_props) in &mut query {
        let mut linear_move = Vec3::ZERO;
        let mut turn_move = 0.0;
        if keys.pressed(KeyCode::KeyI) {
            linear_move.z += 1.0;
        }
        if keys.pressed(KeyCode::KeyK) {
            linear_move.z -= 1.0;
        }
        if keys.pressed(KeyCode::KeyJ) {
            linear_move.x += 1.0;
        }
        if keys.pressed(KeyCode::KeyL) {
            linear_move.x -= 1.0;
        }
        if keys.pressed(KeyCode::KeyZ) {
            turn_move += 1.0;
        }
        if keys.pressed(KeyCode::KeyX) {
            turn_move -= 1.0;
        }

        if linear_move.length_squared() > 0.0 {
            linear_move = linear_move.normalize();
        }

        let target_linear_vel = transform.rotation * linear_move * robot.linear_speed;

        let target_angular_vel = Vec3::new(0.0, turn_move * robot.turn_speed, 0.0);

        let delta_lin = (target_linear_vel - velocity.linvel) * robot.stiffness;
        let delta_ang = (target_angular_vel - velocity.angvel) * robot.stiffness;

        // Doesn't affect y-axis!
        let raw_impulse = Vec3::new(delta_lin.x, 0.0, delta_lin.z) * mass_props.mass;

        impulse.impulse = raw_impulse.clamp_length_max(robot.max_impulse);
        impulse.torque_impulse = delta_ang * mass_props.principal_inertia;
    }
}

fn reset_with_pos(
    commands: &mut Commands,
    query: &Query<Entity, With<ShouldReset>>,
    x_pos: &Vec<f32>,
    z_pos: &Vec<f32>,
) {
    for entity in query {
        commands.entity(entity).despawn();
    }

    let y = (ROBOT_HEIGHT / 2.0) + 0.0001;

    let count = x_pos.len().min(NUM_ARENAS);

    for index in 0..count {
        let x_feet = x_pos[index];
        let z_feet = z_pos[index];

        let arena_offset = Vec3::new(0.0, 0.0, (index as f32) * ARENA_SPACING);

        // Feet!
        let spawn_pos =
            Transform::from_translation(arena_offset + Vec3::new(x_feet * FTM, y, z_feet * FTM))
                .with_rotation(Quat::from_euler(EulerRot::YXZ, 0.0, 0.0, 0.0));

        spawn_robot_objects(spawn_pos, index, commands);
    }
}

#[derive(Component)]
struct ArenaIndex(usize);

#[derive(Component)]
struct ShouldReset;

#[derive(Component)]
struct PhysicsObject;

#[derive(Component)]
struct GoalObject;

#[derive(Component)]
struct DebugObject;

#[derive(Component)]
struct RobotObject {
    linear_speed: f32,
    turn_speed: f32,
    max_impulse: f32,
    snozzle_angle: f32,
    snozzle_pow: f32,
    stiffness: f32,
}

#[derive(Component)]
struct Ball;

// Bevy only supports 15 for tuples, not structs
#[derive(Bundle)]
struct BallBundle {
    rigid_body: RigidBody,
    velocity: Velocity,
    collider: Collider,
    ccd: Ccd,
    restitution: Restitution,
    mass_props: ColliderMassProperties,
    read_mass: ReadMassProperties,
    damping: Damping,
    physics_object: PhysicsObject,
    debug_object: DebugObject,
    transform: Transform,
    ball: Ball,
    resettable: ShouldReset,
    collision_groups: CollisionGroups,
    arena: ArenaIndex,
}

impl BallBundle {
    fn new(position: Vec3, linear_velocity: Vec3, arena: usize) -> Self {
        Self {
            rigid_body: RigidBody::Dynamic,
            velocity: Velocity {
                linvel: linear_velocity,
                angvel: Vec3::new(0.2, 0.4, 0.8),
            },
            collider: Collider::ball(BALL_RAD),
            ccd: Ccd::enabled(),
            restitution: Restitution::coefficient(0.5),
            mass_props: ColliderMassProperties::Mass(BALL_WEIGHT),
            read_mass: ReadMassProperties::default(),
            damping: Damping {
                linear_damping: 0.1,
                angular_damping: 1.0,
            },
            physics_object: PhysicsObject,
            debug_object: DebugObject,
            transform: Transform::from_translation(position),
            ball: Ball,
            resettable: ShouldReset,
            collision_groups: CollisionGroups::new(Group::GROUP_2, Group::ALL ^ Group::GROUP_1),
            arena: ArenaIndex(arena),
        }
    }
}

#[derive(Debug, Resource)]
struct CameraSettings {
    pub orbit_distance: f32,
    pub pitch_speed: f32,
    pub pitch_range: Range<f32>,
    pub yaw_speed: f32,
    pub pan_speed: f32,
}

#[derive(Resource)]
struct TrainingConnection {
    rx_choices: Mutex<ipc::IpcReceiver<ModelChoices>>,
    tx_reward: Mutex<IpcSender<Rewards>>,
    waiting_for_result: bool,
    pending_setup: bool,
    pending_launch: bool,
    index: usize,
    pending_yaws: Vec<f32>,
    pending_powers: Vec<f32>,
    pending_x_pos: Vec<f32>,
    pending_z_pos: Vec<f32>,
    shot_frame_counter: usize,
    collected_rewards: Vec<Option<f32>>,
    finished_count: usize,
    batch_size: Option<usize>,
}

fn setup_graphics(mut commands: Commands, camera_settings: Res<CameraSettings>) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 5.0, -camera_settings.orbit_distance)
            .looking_at(Vec3::ZERO, Vec3::Y),
    ));
    // commands.spawn(PerfUiAllEntries::default());
}

fn setup_physics(mut commands: Commands) {
    for arena_index in 0..NUM_ARENAS {
        let offset = Vec3::new(0.0, 0.0, (arena_index as f32) * ARENA_SPACING);
        spawn_arena_objects(&mut commands, offset);

        spawn_robot_objects(
            Transform::from_xyz(0.0, 0.101, 0.0)
                .with_rotation(Quat::from_euler(EulerRot::YXZ, PI / 4.0, 0.0, 0.0))
                .with_translation(offset + Vec3::new(0.0, 0.101, 0.0)),
            arena_index,
            &mut commands,
        );
    }
}
impl Default for CameraSettings {
    fn default() -> Self {
        let pitch_limit = FRAC_PI_2 - 0.01;
        Self {
            orbit_distance: 5.0,
            pitch_speed: 0.003,
            pitch_range: -pitch_limit..pitch_limit,
            yaw_speed: 0.004,
            pan_speed: 0.005,
        }
    }
}

// Center of panning
#[derive(Debug, Resource, Default)]
struct CameraState {
    pub focus_point: Vec3,
}

fn orbit(
    mut camera: Single<&mut Transform, With<Camera>>,
    camera_settings: Res<CameraSettings>,
    mut camera_state: ResMut<CameraState>,
    mouse_buttons: Res<ButtonInput<MouseButton>>,
    mouse_motion: Res<AccumulatedMouseMotion>,
) {
    let delta = mouse_motion.delta;

    if delta == Vec2::ZERO {
        return;
    }

    if mouse_buttons.pressed(MouseButton::Left) {
        let delta_pitch = delta.y * camera_settings.pitch_speed;
        let delta_yaw = delta.x * camera_settings.yaw_speed;

        let (yaw, pitch, _roll) = camera.rotation.to_euler(EulerRot::YXZ);

        let new_pitch = (pitch + delta_pitch).clamp(
            camera_settings.pitch_range.start,
            camera_settings.pitch_range.end,
        );
        let new_yaw = yaw + delta_yaw;

        camera.rotation = Quat::from_euler(EulerRot::YXZ, new_yaw, new_pitch, 0.0);
    } else if mouse_buttons.pressed(MouseButton::Right) {
        let right = camera.right();
        let up = camera.up();

        let pan_vector = (right * -delta.x * camera_settings.pan_speed)
            + (up * delta.y * camera_settings.pan_speed);

        camera_state.focus_point += pan_vector;
    }

    camera.translation =
        camera_state.focus_point - camera.forward() * camera_settings.orbit_distance;
}

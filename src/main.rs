use bevy::input::mouse::AccumulatedMouseMotion;
use bevy::prelude::*;
use bevy_rapier3d::prelude::*;
use iyes_perf_ui::prelude::*;
use std::{
    f32::consts::{FRAC_PI_2, PI},
    ops::Range,
};

fn main() {
    App::new()
        .init_resource::<CameraSettings>()
        .init_resource::<CameraState>()
        .add_plugins(DefaultPlugins)
        .add_plugins(bevy::diagnostic::FrameTimeDiagnosticsPlugin::default())
        .add_plugins(bevy::diagnostic::EntityCountDiagnosticsPlugin)
        .add_plugins(bevy::diagnostic::SystemInformationDiagnosticsPlugin)
        .add_plugins(bevy::render::diagnostic::RenderDiagnosticsPlugin)
        .add_plugins(PerfUiPlugin)
        .add_plugins(RapierPhysicsPlugin::<NoUserData>::default())
        .add_plugins(RapierDebugRenderPlugin::default())
        // Calculates collisions at 60hz
        .insert_resource(Time::<Fixed>::from_seconds(1.0 / 60.0))
        .add_systems(Startup, setup_graphics)
        .add_systems(Startup, setup_physics)
        // .add_systems(Update, print_debug)
        .add_systems(Update, take_out)
        .add_systems(FixedUpdate, orbit)
        .add_systems(Update, reset_simulation)
        .add_systems(FixedUpdate, move_robot)
        .run();
}

fn setup_physics(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    spawn_scene_objects(&mut commands, &mut meshes, &mut materials);
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

fn spawn_scene_objects(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
) {
    // Arena
    commands.spawn((
        Collider::cuboid(WALL_LENGTH / 2.0, 0.001, WALL_LENGTH / 2.0),
        Transform::from_xyz(0.0, -0.001, 0.0),
        PhysicsObject,
        Mesh3d(meshes.add(Cuboid::new(
            WALL_LENGTH / 2.0 * 2.0,
            0.002,
            WALL_LENGTH / 2.0 * 2.0,
        ))),
        MeshMaterial3d(materials.add(Color::srgb(0.8, 0.8, 0.8))),
    ));

    commands.spawn((
        Collider::cuboid(0.001, WALL_HEIGHT / 2.0, WALL_LENGTH / 2.0),
        Transform::from_xyz(WALL_LENGTH / 2.0, WALL_HEIGHT / 2.0, 0.0),
        PhysicsObject,
    ));

    commands.spawn((
        Collider::cuboid(0.001, WALL_HEIGHT / 2.0, WALL_LENGTH / 2.0),
        Transform::from_xyz(-WALL_LENGTH / 2.0, WALL_HEIGHT / 2.0, 0.0),
        PhysicsObject,
    ));

    commands.spawn((
        Collider::cuboid(WALL_LENGTH / 2.0, WALL_HEIGHT / 2.0, 0.001),
        Transform::from_xyz(0.0, WALL_HEIGHT / 2.0, WALL_LENGTH / 2.0),
        PhysicsObject,
    ));

    commands.spawn((
        Collider::cuboid(WALL_LENGTH / 2.0, WALL_HEIGHT / 2.0, 0.001),
        Transform::from_xyz(0.0, WALL_HEIGHT / 2.0, -WALL_LENGTH / 2.0),
        PhysicsObject,
    ));

    commands.spawn((
        Collider::cuboid(0.495, GOAL_HEIGHT / 2.0, 0.001),
        Transform::from_xyz(1.479, GOAL_HEIGHT / 2.0, 1.479).with_rotation(Quat::from_euler(
            EulerRot::YXZ,
            PI / 4.0,
            0.0,
            0.0,
        )),
        PhysicsObject,
    ));

    commands.spawn((
        Collider::cuboid(0.35, GOAL_HEIGHT / 2.0, 0.001),
        Transform::from_xyz(
            (WALL_LENGTH / 2.0) - 0.35,
            GOAL_HEIGHT / 2.0,
            WALL_LENGTH / 2.0,
        ),
        PhysicsObject,
    ));

    commands.spawn((
        Collider::triangle(
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(0.70, 0.0, 0.0),
            Vec3::new(0.70, 0.375, 0.0),
        ),
        Transform::from_xyz((WALL_LENGTH / 2.0) - 0.7, GOAL_HEIGHT, WALL_LENGTH / 2.0),
        PhysicsObject,
    ));

    commands.spawn((
        Collider::cuboid(0.001, GOAL_HEIGHT / 2.0, 0.35),
        Transform::from_xyz(
            WALL_LENGTH / 2.0,
            GOAL_HEIGHT / 2.0,
            (WALL_LENGTH / 2.0) - 0.35,
        ),
        PhysicsObject,
    ));

    commands.spawn((
        Collider::triangle(
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 0.70),
            Vec3::new(0.0, 0.375, 0.70),
        ),
        Transform::from_xyz(WALL_LENGTH / 2.0, GOAL_HEIGHT, (WALL_LENGTH / 2.0) - 0.7),
        PhysicsObject,
    ));

    // Robot
    commands
        .spawn((
            Velocity::default(),
            ExternalImpulse::default(),
            Ccd::enabled(),
            LockedAxes::ROTATION_LOCKED_X | LockedAxes::ROTATION_LOCKED_Z,
            RigidBody::Dynamic,
            Collider::cuboid(ROBOT_LENGTH / 2.0, ROBOT_HEIGHT / 2.0, ROBOT_LENGTH / 2.0),
            Transform::from_xyz(0.0, 0.101, 0.0),
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
                snozzle_angle: 70.0,
                snozzle_pow: 6.5, // m/s,
                stiffness: 0.5,
            },
            Mesh3d(meshes.add(Cuboid::new(ROBOT_LENGTH, ROBOT_HEIGHT, ROBOT_LENGTH))),
            MeshMaterial3d(materials.add(GREEN)),
        ))
        .insert((PhysicsObject, DebugObject));
}

fn _print_debug(
    query: Query<(Option<&RobotObject>, &ReadMassProperties, &Velocity), With<DebugObject>>,
) {
    for (robot_comp, mass_props, velocity) in &query {
        let name = if robot_comp.is_some() {
            "Robot"
        } else {
            "Ball"
        };

        println!(
            "{} Stats: Mass: {:.2} kg | velocity: {:.2}",
            name, mass_props.mass, velocity.linvel
        );
    }
}

const GREEN: bevy::prelude::Color = Color::srgb(0.0, 2.0, 0.0);

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
}

impl BallBundle {
    fn new(position: Vec3, linear_velocity: Vec3) -> Self {
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
        }
    }
}

fn take_out(
    mut commands: Commands,
    keys: Res<ButtonInput<KeyCode>>,
    mut query: Query<(&Transform, &mut RobotObject)>,
) {
    for (transform, mut robot) in &mut query {
        let angle_increment: f32 = 1.0;

        if keys.pressed(KeyCode::ArrowDown) {
            robot.snozzle_angle = (robot.snozzle_angle - angle_increment).max(0.0);
        }
        if keys.pressed(KeyCode::ArrowUp) {
            robot.snozzle_angle = (robot.snozzle_angle + angle_increment).min(90.0);
        }

        let pow_increment: f32 = 0.01;

        if keys.pressed(KeyCode::ArrowLeft) {
            robot.snozzle_pow = (robot.snozzle_pow - pow_increment).max(0.0);
        }
        if keys.pressed(KeyCode::ArrowRight) {
            robot.snozzle_pow = (robot.snozzle_pow + pow_increment).min(8.0);
        }

        let pitch_radians = -robot.snozzle_angle.to_radians();

        let pitch_rotation = Quat::from_rotation_x(pitch_radians);

        let final_rotation = transform.rotation * pitch_rotation;

        let forward_dir = final_rotation * Vec3::Z;
        let shoot_velocity = forward_dir * robot.snozzle_pow;

        let spawn_pos = transform.translation + (Vec3::Y * ROBOT_HEIGHT);

        // let spawn_pos = transform.translation + (Vec3::Y * ROBOT_HEIGHT) + (forward_dir * (ROBOT_LENGTH / 1.5));

        if keys.just_pressed(KeyCode::KeyS) {
            commands.spawn(BallBundle::new(spawn_pos, shoot_velocity));
        }
        if keys.pressed(KeyCode::KeyA) {
            commands.spawn(BallBundle::new(spawn_pos, shoot_velocity));
        }
    }
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

fn reset_simulation(
    mut commands: Commands,
    keys: Res<ButtonInput<KeyCode>>,
    query: Query<Entity, With<PhysicsObject>>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    if keys.just_pressed(KeyCode::KeyR) {
        for entity in &query {
            commands.entity(entity).despawn();
        }

        spawn_scene_objects(&mut commands, &mut meshes, &mut materials);
    }
}

#[derive(Component)]
struct PhysicsObject;

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

#[derive(Debug, Resource)]
struct CameraSettings {
    pub orbit_distance: f32,
    pub pitch_speed: f32,
    pub pitch_range: Range<f32>,
    pub yaw_speed: f32,
    pub pan_speed: f32,
}

fn setup_graphics(mut commands: Commands, camera_settings: Res<CameraSettings>) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 5.0, -camera_settings.orbit_distance)
            .looking_at(Vec3::ZERO, Vec3::Y),
    ));
    // commands.spawn(PerfUiAllEntries::default());
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

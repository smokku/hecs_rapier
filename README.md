# physics engine _for hecs_

Port of `bevy_rapier2d` to hecs ECS.

## WARNING

- This code requires hecs fork with entity change tracking: <https://github.com/smokku/hecs.git>
- Unfinished. physics_hooks are not yet ported.

## Example

```rust
use hecs_rapier as physics;
use hecs::World;

let mut world = World::new();

/* Create structures necessary for the simulation. */
let gravity = vector![0.0, -9.81];
let integration_parameters = physics::IntegrationParameters::default();
let mut physics_pipeline = physics::PhysicsPipeline::new();
let mut island_manager = physics::IslandManager::new();
let mut broad_phase = physics::BroadPhase::new();
let mut narrow_phase = physics::NarrowPhase::new();
let mut joint_set = physics::JointSet::new();
let mut joints_entity_map = physics::JointsEntityMap::default();
let mut ccd_solver = physics::CCDSolver::new();
// let physics_hooks = ();
let event_handler = ();
let mut modification_tracker = physics::ModificationTracker::default();

while running {
        physics::attach_bodies_and_colliders(&mut world);
        physics::create_joints_system(&mut self.world, &mut joint_set, &mut joints_entity_map);
        physics::finalize_collider_attach_to_bodies(&mut world, &mut modification_tracker);


        physics::prepare_step(&mut world, &mut modification_tracker);
        if time_to_step {

            physics::step_world(
                &mut world,
                &gravity,
                &integration_parameters,
                &mut physics_pipeline,
                &mut modification_tracker,
                &mut island_manager,
                &mut broad_phase,
                &mut narrow_phase,
                &mut joint_set,
                &mut joints_entity_map,
                &mut ccd_solver,
                &event_handler,
            );

        }


        physics::collect_removals(&mut world, &mut modification_tracker);

        next_frame().await;
        world.clear_trackers();
}
```

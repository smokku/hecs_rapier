use super::*;
use crate::resources::ModificationTracker;
use hecs::{Added, Without, World};

pub fn prepare_step(world: &mut World, modification_tracker: &mut ModificationTracker) {
    modification_tracker.detect_removals(world);
    modification_tracker.detect_modifications(world);
}

/// System responsible for performing one timestep of the physics world.
#[allow(clippy::too_many_arguments)]
pub fn step_world(
    world: &mut World,
    gravity: &Vector<Real>,
    integration_parameters: &IntegrationParameters,
    physics_pipeline: &mut PhysicsPipeline,
    modification_tracker: &mut ModificationTracker,
    island_manager: &mut IslandManager,
    broad_phase: &mut BroadPhase,
    narrow_phase: &mut NarrowPhase,
    joint_set: &mut JointSet,
    ccd_solver: &mut CCDSolver,
    // physics_hooks: &dyn PhysicsHooks<RigidBodyComponentsSet, ColliderComponentsSet>,
    event_handler: &dyn EventHandler,
) {
    // println!("step");
    use std::mem::take;

    let physics_hooks = ();

    let mut rigid_body_components_set = RigidBodyComponentsSet(world);
    let mut collider_components_set = ColliderComponentsSet(world);

    let cleanup_entities = modification_tracker.propagate_removals(
        island_manager,
        &mut rigid_body_components_set,
        // &mut joints,
        // &mut joints_entity_map,
    );
    island_manager.cleanup_removed_rigid_bodies(&mut rigid_body_components_set);

    physics_pipeline.step_generic(
        gravity,
        integration_parameters,
        island_manager,
        broad_phase,
        narrow_phase,
        &mut rigid_body_components_set,
        &mut collider_components_set,
        &mut take(&mut modification_tracker.modified_bodies),
        &mut take(&mut modification_tracker.modified_colliders),
        &mut take(&mut modification_tracker.removed_colliders),
        joint_set,
        ccd_solver,
        &physics_hooks,
        event_handler,
    );

    for entity in cleanup_entities {
        let _ = world.remove::<ColliderBundle>(entity);
        let _ = world.remove_one::<ColliderParent>(entity);
    }
    modification_tracker.clear_modified_and_removed();
}

/// System responsible for creating a Rapier rigid-body and collider from their
/// builder resources.
pub fn attach_bodies_and_colliders(world: &mut World) {
    // println!("attach_bodies_and_colliders");
    let mut co_parents = Vec::new();
    'outer: for (collider_entity, co_pos) in world
        .query::<Without<
            ColliderParent,
            // Colliders.
            &ColliderPosition,
        >>()
        .iter()
    {
        // Find the closest ancestor (possibly the same entity) with a body
        let mut body_entity = collider_entity;
        loop {
            if world.get::<RigidBodyPosition>(body_entity).is_ok() {
                // Found it!
                break;
            } else if let Ok(parent_entity) = world.get::<Parent>(body_entity) {
                body_entity = **parent_entity;
            } else {
                continue 'outer;
            }
        }

        let co_parent = ColliderParent {
            pos_wrt_parent: co_pos.0,
            handle: body_entity.handle(),
        };
        co_parents.push((collider_entity, co_parent));
    }
    for (collider_entity, co_parent) in co_parents.drain(..) {
        world.insert_one(collider_entity, co_parent).unwrap();
    }
}

/// System responsible for creating a Rapier rigid-body and collider from their
/// builder resources.
pub fn finalize_collider_attach_to_bodies(
    world: &mut World,
    modification_tracker: &mut ModificationTracker,
) {
    // println!("finalize_collider_attach_to_bodies");

    for (
        collider_entity,
        (
            mut co_changes,
            mut co_bf_data,
            mut co_pos,
            co_shape,
            co_mprops,
            co_parent,
            _added_colider_parent,
        ),
    ) in world
        .query::<(
            // Collider.
            &mut ColliderChanges,
            &mut ColliderBroadPhaseData,
            &mut ColliderPosition,
            &ColliderShape,
            &ColliderMassProps,
            &ColliderParent,
            Added<ColliderParent>,
        )>()
        .iter()
        .filter(|(_e, (_, _, _, _, _, _, added))| *added)
    {
        let mut body_query = world.query_one::<(
                // Rigid-bodies.
                &mut RigidBodyChanges,
                &mut RigidBodyCcd,
                &mut RigidBodyColliders,
                &mut RigidBodyMassProps,
                &RigidBodyPosition,
            )>(co_parent.handle.entity()).unwrap();
        if let Some((mut rb_changes, mut rb_ccd, mut rb_colliders, mut rb_mprops, rb_pos)) =
            body_query.get()
        {
            // Contract:
            // - Reset collider's references.
            // - Set collider's parent handle.
            // - Attach the collider to the body.

            // Update the modification tracker.
            // NOTE: this must be done before the `.attach_collider` because
            //       `.attach_collider` will set the `MODIFIED` flag.

            if !rb_changes.contains(RigidBodyChanges::MODIFIED) {
                modification_tracker.modified_bodies.push(co_parent.handle);
            }

            modification_tracker
                .body_colliders
                .entry(co_parent.handle)
                .or_insert_with(Vec::new)
                .push(collider_entity.handle());
            modification_tracker
                .colliders_parent
                .insert(collider_entity.handle(), co_parent.handle);

            *co_changes = ColliderChanges::default();
            *co_bf_data = ColliderBroadPhaseData::default();
            rb_colliders.attach_collider(
                &mut rb_changes,
                &mut rb_ccd,
                &mut rb_mprops,
                rb_pos,
                collider_entity.handle(),
                &mut co_pos,
                co_parent,
                co_shape,
                co_mprops,
            );
        }
    }
}

/// System responsible for collecting the entities with removed rigid-bodies, colliders,
/// or joints.
pub fn collect_removals(world: &mut World, modification_tracker: &mut ModificationTracker) {
    modification_tracker.detect_removals(world);
}

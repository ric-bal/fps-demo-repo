using System;
using KinematicCharacterController;
using UnityEngine;

public struct CharacterInput
{
    public Quaternion Rotation;
    public Vector2 Move;
    public bool Jump;
    public bool JumpSustain;
    public bool SlamSlide;
}

public enum Stance
{
    Stand, Slam, Slide
}

public struct CharacterState
{
    public bool Grounded;
    public Stance Stance;
    public Vector3 Velocity;
}

public class PlayerCharacter : MonoBehaviour, ICharacterController
{
    [SerializeField] private KinematicCharacterMotor motor;
    [SerializeField] private Transform root;
    [SerializeField] private Transform cameraTarget;

    [Space]

    [SerializeField] private float walkSpeed = 20f;
    [SerializeField] private float walkResponse = 25f; // smoothing acceleration

    [Space]

    [SerializeField] private float airSpeed = 15f;
    [SerializeField] private float airAcceleration = 70f;

    [Space]

    [SerializeField] private float jumpSpeed = 20f;
    [SerializeField] private float horizontalAirHopSpeed = 25f;
    [Range(0f, 1f)]
    [SerializeField] private float jumpSustainGravity = 0.4f;
    [SerializeField] private float gravity = -90f;

    [Space]

    [SerializeField] private float hoverGravity = 0.05f;
    [SerializeField] private float _hoverTimer = 2f;
    [SerializeField] private float _currentHoverTimer = 1f;

    [Space]

    [SerializeField] private float slideStartSpeed = 25f;
    [SerializeField] private float slideEndSpeed = 1f;
    [SerializeField] private float slideFriction = 0.6f;
    [SerializeField] private float slideSteerAcceleration = 5f;
    [SerializeField] private float slideGravity = -90f; // for slope acceleration


    [SerializeField] private float slideHeight = 0.6f;
    [SerializeField] private float standHeight = 2f;
    [SerializeField] private float slideHeightResponse = 20f; // smoothing acceleration

    [Range(0f, 1f)]
    [SerializeField] private float standCameraTargetHeight = 0.4f; // relative to character capsule collider (height of 2 units)
    [Range(0f, 1f)]
    [SerializeField] private float slideCameraTargetHeight = 0.05f;

    [Space]

    private CharacterState _state;
    private CharacterState _lastState;
    private CharacterState _tempState;

    private Quaternion _requestedRotation;
    private Vector3 _requestedMovement;
    private bool _requestedJump;
    private bool _requestedSustainedJump;
    private bool _requestedSlamSlide;

    private float _currentVelocityMag = 0f;

    private bool _lockSlide = false;
    private bool _cancelSlide = false;

    private Collider[] _unslideOverlapResults;


    public void Initialize()
    {
        _state.Stance = Stance.Stand;
        _lastState = _state;
        _unslideOverlapResults = new Collider[8];

        motor.CharacterController = this;
    }


    public void UpdateInput(CharacterInput input)
    {
        _requestedRotation = input.Rotation;

        _requestedMovement =  new Vector3(input.Move.x, 0f, input.Move.y);
        _requestedMovement =  Vector3.ClampMagnitude(_requestedMovement, 1f);
        _requestedMovement =  input.Rotation * _requestedMovement;

        _requestedJump =  _requestedJump || input.Jump;
        _requestedSustainedJump = input.JumpSustain;

        if (_lastState.Stance is Stance.Slide && _state.Stance is Stance.Stand)
        {
            _lockSlide = true;
            _requestedSlamSlide = false;
        }
        if (_lockSlide && !input.SlamSlide)
        {
            _lockSlide = false;
        }

        if (!_lockSlide)
        {
            _requestedSlamSlide = input.SlamSlide;
        }
    }


    public void UpdateBody(float deltaTime)
    {
        var currentHeight = motor.Capsule.height;
        var normalisedHeight = currentHeight / standHeight;

        var cameraTargetHeight =  currentHeight * (_state.Stance is Stance.Stand ? standCameraTargetHeight : slideCameraTargetHeight);
        var rootTargetScale = new Vector3(1f, normalisedHeight, 1f);

        cameraTarget.localPosition = Vector3.Lerp
        (
            a: cameraTarget.localPosition,
            b: new Vector3(0f, cameraTargetHeight, 0f),
            t: 1f - Mathf.Exp(-slideHeightResponse * deltaTime)
        );

        root.localScale = Vector3.Lerp
        (
            a: root.localScale,
            b: rootTargetScale,
            t: 1f - Mathf.Exp(-slideHeightResponse * deltaTime)
        );
    }


    public void UpdateRotation(ref Quaternion currentRotation, float deltaTime)
    {
        // ProjectOnPlane(vector, normal to plane to be projected onto)
            // (_requestedRotation * Vector3.forward) is the player's relative forward
            // (motor.CharacterUp) is the player's relative up, so (0, 0, 1) if walking on flat ground
        // flatten (project) forward vector onto plane for vector on xz plane (assuming up is pos y)

        var forward = Vector3.ProjectOnPlane 
        (
            _requestedRotation * Vector3.forward, 
            motor.CharacterUp
        );
        currentRotation = Quaternion.LookRotation(forward, motor.CharacterUp);
    }


    public void UpdateVelocity(ref Vector3 currentVelocity, float deltaTime)
    {
        var currentPlanarVelocity = Vector3.ProjectOnPlane
        (
            vector: currentVelocity,
            planeNormal: motor.CharacterUp
        );

        // on the ground
        if (motor.GroundingStatus.IsStableOnGround)
        {
            // requested movement direction to angle of surface being walked on
            var groundedMovement = motor.GetDirectionTangentToSurface
            (
                direction: _requestedMovement,
                surfaceNormal: motor.GroundingStatus.GroundNormal
            ) * _requestedMovement.magnitude;

            // sliding
            {
                var moving = groundedMovement.sqrMagnitude > 0f;
                var slideStanced = _state.Stance is Stance.Slide;
                var wasStanding = _lastState.Stance is Stance.Stand;
                var wasInAir = false; //!_lastState.Grounded;
                if (moving && slideStanced && (wasStanding || wasInAir))
                {
                    _state.Stance = Stance.Slide;

                    // land on stable ground, character motor prohects velocity onto flat ground plane, stops sliding
                    // KinematicCharacterMotor.HandleVelocityProjection()
                    // we want sliding
                    // reproject last frames falling velocity onto ground normal to slide
                    if (wasInAir)
                    {
                        currentVelocity = Vector3.ProjectOnPlane
                        (
                            vector: _lastState.Velocity,
                            planeNormal: motor.GroundingStatus.GroundNormal
                        );
                    }

                    var slideSpeed = Mathf.Max(slideStartSpeed, currentVelocity.magnitude);
                    currentVelocity = motor.GetDirectionTangentToSurface
                    (
                        direction: currentVelocity,
                        surfaceNormal: motor.GroundingStatus.GroundNormal
                    ) * slideSpeed;
                }
            }

            // movement
            if (_state.Stance is Stance.Stand or Stance.Slam)
            {
                var targetVelocity = groundedMovement * walkSpeed;

                currentVelocity = Vector3.Lerp
                (
                    a: currentVelocity,
                    b: targetVelocity,
                    t: 1f - Mathf.Exp(-walkResponse * deltaTime)
                );
            }
            // must be sliding, keep sliding
            else
            {
                currentVelocity -= currentVelocity * (slideFriction * deltaTime);

                // slope acceleration
                {
                    var force = Vector3.ProjectOnPlane
                    (
                        vector: -motor.CharacterUp,
                        planeNormal: motor.GroundingStatus.GroundNormal
                    ) * slideGravity;

                    currentVelocity -= force * deltaTime;
                }

                // steering
                {
                    var currentSpeed = currentVelocity.magnitude;
                    var targetVelocity = groundedMovement * currentSpeed;

                    if (groundedMovement == Vector3.zero) // no directional keys pressed, still give boost for consistency
                    {
                        var forward = Vector3.ProjectOnPlane 
                        (
                            _requestedRotation * Vector3.forward, 
                            motor.CharacterUp
                        ).normalized;

                        targetVelocity = forward * currentSpeed;
                    }
                    

                    var steerForce = (targetVelocity - currentVelocity) * slideSteerAcceleration * deltaTime;

                    currentVelocity += steerForce;
                    currentVelocity = Vector3.ClampMagnitude(currentVelocity, currentSpeed);
                }

                if (currentVelocity.magnitude < slideEndSpeed)
                {
                    if (_requestedSlamSlide)
                    {
                        _cancelSlide = true;
                    }
                    _state.Stance = Stance.Stand;
                }
            }
        }
        // in the air
        else
        {
            // in-air movement

            // MANAGE SLIDING HERE

            // if (_requestedSlamSlide || _state.Stance is Stance.Slam)
            // {
            //     _state.Stance = Stance.Slam;
            //     _requestedSlamSlide = false;

            //     if (motor.GroundingStatus.IsStableOnGround)
            //     {
            //         _state.Stance = Stance.Stand;
            //     }
            // }

            if (_requestedMovement.sqrMagnitude > 0f)
            {
                // requested movement on XZ plane
                var planarMovement = Vector3.ProjectOnPlane
                (
                    vector: _requestedMovement,
                    planeNormal: motor.CharacterUp
                ) * _requestedMovement.magnitude;

                // movement force
                var movementForce = airAcceleration * deltaTime * planarMovement;

                // if moving slower than max air speed, treat movement force as a steering force
                if (currentPlanarVelocity.magnitude < airSpeed)
                {
                    var targetPlanarVelocity = currentPlanarVelocity + movementForce;
                    targetPlanarVelocity = Vector3.ClampMagnitude(targetPlanarVelocity, airSpeed); // limit target velocity to air speed

                    movementForce = targetPlanarVelocity - currentPlanarVelocity;
                }
                // otherwise, nerf movement force when it is in the direction of the current planar velocity
                // to prevent accelerating beyond max air speed
                else if (Vector3.Dot(currentPlanarVelocity, movementForce) > 0f)
                {
                    // project movement force onto plane whose normal is current planar velocity, negates any force in same direction as current velocity
                    var constrainedMovementForce = Vector3.ProjectOnPlane
                    (
                        vector: movementForce,
                        planeNormal: currentPlanarVelocity.normalized
                    );

                    movementForce = constrainedMovementForce;
                }

                // prevent air-climbing up steep slopes
                if (motor.GroundingStatus.FoundAnyGround)
                {
                    if (Vector3.Dot(movementForce, currentVelocity + movementForce) > 0f)
                    {
                        var obstructionNormal = Vector3.Cross // vector perpendicular to 2 given vectors
                        (
                            motor.CharacterUp,
                            Vector3.Cross
                            (
                                motor.CharacterUp,
                                motor.GroundingStatus.GroundNormal
                            )
                        ).normalized; // normal of obstruction, normalised

                        movementForce = Vector3.ProjectOnPlane(movementForce, obstructionNormal); // removes any component of movement force aligned with obstruction normal
                    }
                }

                // steering
                currentVelocity += movementForce;
            }

            // gravity
            var effectiveGravity = gravity;
            var verticalSpeed = Vector3.Dot(currentVelocity, motor.CharacterUp); 
            if (_requestedSustainedJump)
            {
                _currentHoverTimer += deltaTime;

                // effectiveGravity is multiplied, not set directly
                // sustained jump
                if (verticalSpeed > 0f)
                {
                    _currentHoverTimer = 0f; // reset hover timer
                    effectiveGravity *= jumpSustainGravity;
                }
                // hovering
                else if (_currentHoverTimer < _hoverTimer)
                {
                    effectiveGravity *= hoverGravity;
                }
                else
                {
                    effectiveGravity = Mathf.Lerp
                    (
                        a: effectiveGravity * hoverGravity,
                        b: effectiveGravity,
                        t: 1f - Mathf.Exp(-30f * deltaTime)
                    );
                }
            }
            currentVelocity += deltaTime * effectiveGravity * motor.CharacterUp;


        }
 
        // press jump
        if (_requestedJump)
        {
            _requestedJump = false;
            _requestedSlamSlide = false; 

            motor.ForceUnground(time: 0.1f);

            var requestedAirVelocity = motor.GetDirectionTangentToSurface // value 0 to 1
            (
                direction: _requestedMovement,
                surfaceNormal: motor.CharacterUp
            );

            var currentVerticalSpeed = Vector3.Dot(currentVelocity, motor.CharacterUp); // calculates relative upward speed

            // ensures movements do not reduce current velocity faster than jump/air hop speed
            var targetVerticalSpeed = Mathf.Max(currentVerticalSpeed, jumpSpeed); 
            var targetHorizontalSpeed = Mathf.Max(currentPlanarVelocity.magnitude, _lastState.Grounded ? jumpSpeed : horizontalAirHopSpeed);

            // cancels velocity on XZ plane if player wants to move in XZ plane, otherwise continue current velocity 
            if (requestedAirVelocity != Vector3.zero)
            {
                currentVelocity = Vector3.Project(currentVelocity, motor.CharacterUp);
            }

            // adjusts the velocity in each direction
            currentVelocity += motor.CharacterUp * (targetVerticalSpeed - currentVerticalSpeed); // (0, some value, 0)
            currentVelocity += requestedAirVelocity * targetHorizontalSpeed;

            // ALTERNATE VERSION - slow down XZ velocity if player air hops in place (player does not want to move in XZ plane)
            // at bottom
        }

        _currentVelocityMag = currentVelocity.magnitude;
    }


    public void BeforeCharacterUpdate(float deltaTime)
    {
        _tempState = _state;
        
        // slide: smaller character
        if (_requestedSlamSlide && _state.Grounded && _currentVelocityMag > slideEndSpeed)
        {
            _state.Stance = Stance.Slide;
            motor.SetCapsuleDimensions
            (
                radius: motor.Capsule.radius,
                height: slideHeight,
                yOffset: slideHeight * -0f
            );

            if (_lastState.Stance is Stance.Stand) // as soon as slide starts, move character down so they touch the floor
            {
                transform.Translate(-((standHeight - slideHeight) * 0.5f * motor.CharacterUp));
            }
        } 
    }


    public void PostGroundingUpdate(float deltaTime)
    {
        if (!motor.GroundingStatus.IsStableOnGround)
        {
            if (_state.Stance is Stance.Slide && !_requestedSlamSlide)
            {
                _state.Stance = Stance.Stand;
            }
        }
    }


    public void AfterCharacterUpdate(float deltaTime)
    {
        // slide: normal size character
        if ((!_requestedSlamSlide || _cancelSlide) && _state.Grounded)
        {
            _cancelSlide = false;

            motor.SetCapsuleDimensions
            (
                radius: motor.Capsule.radius,
                height: standHeight,
                yOffset: 0f
            );

            // only allow standing up after sliding if not colliding with any other objects
            if (motor.CharacterOverlap(motor.TransientPosition, motor.TransientRotation, _unslideOverlapResults, motor.CollidableLayers, QueryTriggerInteraction.Ignore) > 1)
            {
                _requestedSlamSlide = true;
                // reslide
                _state.Stance = Stance.Slide;
                motor.SetCapsuleDimensions
                (
                    radius: motor.Capsule.radius,
                    height: slideHeight,
                    yOffset: 0f 
                );
            }
            else
            {
                _state.Stance = Stance.Stand;
            }
        } 

        _state.Grounded = motor.GroundingStatus.IsStableOnGround;
        _state.Velocity = motor.Velocity;
        _lastState = _tempState; 
    }


    public bool IsColliderValidForCollisions(Collider coll)
    {
        //throw new System.NotImplementedException();
        return true;
    }


    public void OnGroundHit(Collider hitCollider, Vector3 hitNormal, Vector3 hitPoint, ref HitStabilityReport hitStabilityReport)
    {
        //throw new System.NotImplementedException();
    }


    public void OnMovementHit(Collider hitCollider, Vector3 hitNormal, Vector3 hitPoint, ref HitStabilityReport hitStabilityReport)
    {
        //throw new System.NotImplementedException();
    }


    public void ProcessHitStabilityReport(Collider hitCollider, Vector3 hitNormal, Vector3 hitPoint, Vector3 atCharacterPosition, Quaternion atCharacterRotation, ref HitStabilityReport hitStabilityReport)
    {
        //throw new System.NotImplementedException();
    }


    public void OnDiscreteCollisionDetected(Collider hitCollider)
    {
        //throw new System.NotImplementedException();
    }


    public Transform GetCameraTarget() => cameraTarget;


    public void SetPosition(Vector3 position, bool killVelocity = true)
    {
        motor.SetPosition(position);
        if (killVelocity)
        {
            motor.BaseVelocity = Vector3.zero;
        }
    }
}


/* ALTERNATE VERSION (AIRHOP) - slow down XZ velocity if player air hops in place (player does not want to move in XZ plane)

var requestedAirVelocity = motor.GetDirectionTangentToSurface // value 0 to 1
(
    direction: _requestedMovement,
    surfaceNormal: motor.CharacterUp
);

var currentVerticalSpeed = Vector3.Dot(currentVelocity, motor.CharacterUp); // calculates relative upward speed

// ensures movements do not reduce current velocity faster than jump/air hop speed
var targetVerticalSpeed = Mathf.Max(currentVerticalSpeed, jumpSpeed); 
var targetHorizontalSpeed = Mathf.Max(currentPlanarVelocity.magnitude, _lastState.Grounded ? jumpSpeed : horizontalAirHopSpeed);

// cancels velocity on XZ plane
currentVelocity = Vector3.Project(currentVelocity, motor.CharacterUp);

// give upward velocity for jump
currentVelocity += motor.CharacterUp * (targetVerticalSpeed - currentVerticalSpeed); // (0, some value, 0)

// player wants to move in a new direciton
if (requestedAirVelocity != Vector3.zero)
{
    currentVelocity += requestedAirVelocity * targetHorizontalSpeed;
}
// continue moving in old direction
else
{
    currentVelocity += currentPlanarVelocity * 0.3f;
}
*/

/* OLD VERSION (AIRHOP) - velocity on XZ plane is not affected by air hops

var currentVerticalSpeed = Vector3.Dot(currentVelocity, motor.CharacterUp); 
var targetVerticalSpeed = Mathf.Max(currentVerticalSpeed, jumpSpeed);

currentVelocity += motor.CharacterUp * (targetVerticalSpeed - currentVerticalSpeed);
*/
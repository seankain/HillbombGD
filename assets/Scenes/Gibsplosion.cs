using Godot;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

public partial class Gibsplosion : Node3D
{
    [Export]
    public Gib[] GibComponents;
    [Export]
    public float GibsplosionForce = 50f;

    [Export]
    public GpuParticles3D ParticleEmitter;

    private Transform3D[] ComponentStartLocations;

    private bool Ungibbing = false;

    public override void _Ready()
    {
        ComponentStartLocations = new Transform3D[GibComponents.Length];
        for (var i = 0; i < GibComponents.Length; i++)
        {
            ComponentStartLocations[i] = new Transform3D(GibComponents[i].GlobalTransform.Basis, GibComponents[i].GlobalTransform.Origin);
        }

    }

    public void Disable()
    {
        foreach (var g in GibComponents)
        {
            g.Disable();
        }
        this.ProcessMode = ProcessModeEnum.Disabled;
        this.SetPhysicsProcess(false);
    }

    public void Enable()
    {
        this.ProcessMode = ProcessModeEnum.Always;
        this.SetPhysicsProcess(true);
        foreach (var g in GibComponents)
        {
            g.Enable();
        }

    }


    public void Gibsplode()
    {
        foreach (var c in GibComponents)
        {
            c.GetChildByType<RigidBody3D>().ApplyImpulse((this.Position + c.Position) * GibsplosionForce, this.Position);
        }
    }

    public void Gibsplode(float forceMultiplier, Vector3 velocity, Vector3 angularVelocity)
    {
        foreach (var c in GibComponents)
        {
            c.ApplyImpulse((this.Position + c.Position) * forceMultiplier, this.Position);
            c.AngularVelocity = angularVelocity;
            c.LinearVelocity = velocity;
        }
        if (ParticleEmitter != null)
        {
            ParticleEmitter.Emitting = true;
            ParticleEmitter.OneShot = true;
        }
    }

    public void Teleport(Vector3 position)
    {
        this.GlobalPosition = position;
        Ungib();
    }

    public void Ungib()
    {
        for (var i = 0; i < GibComponents.Length; i++)
        {
            GibComponents[i].Reset();
        }
    }

}

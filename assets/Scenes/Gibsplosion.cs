using Godot;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

public partial class Gibsplosion : Node3D
{
    [Export]
    public Node3D[] GibComponents;
    [Export]
    public float GibsplosionForce = 50f;

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
            var rb = c.GetChildByType<RigidBody3D>();
            rb.ApplyImpulse((this.Position + c.Position) * forceMultiplier, this.Position);
            rb.AngularVelocity = angularVelocity;
            rb.LinearVelocity = velocity;
        }
    }

    public void Ungib()
    {
        for (var i = 0; i < GibComponents.Length; i++)
        {
            GD.Print(GibComponents[i].GlobalTransform);
            var rb = GibComponents[i].GetChildByType<RigidBody3D>();
            rb.LinearVelocity = Vector3.Zero;
            rb.AngularVelocity = Vector3.Zero;
            rb.FreezeMode = RigidBody3D.FreezeModeEnum.Kinematic;
            rb.LockRotation = true;
            rb.Freeze = true;
            GibComponents[i].GlobalTransform = new Transform3D(ComponentStartLocations[i].Basis, ComponentStartLocations[i].Origin);
            GD.Print(GibComponents[i].GlobalTransform);
            rb.LockRotation = false;
            rb.Freeze = false;
        }
    }

}

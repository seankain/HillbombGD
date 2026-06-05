using Godot;
using System;
using System.Collections.Generic;

public partial class TrafficPool : Node3D
{
	[Export]
	public PackedScene NpcCarScene;

	[Export]
	public int PoolSize = 20;

	private Stack<NpcCar> _available;
	private HashSet<NpcCar> _active;

	public override void _Ready()
	{
		_available = new Stack<NpcCar>(PoolSize);
		_active = new HashSet<NpcCar>();

		for (int i = 0; i < PoolSize; i++)
		{
			var carNode = NpcCarScene.Instantiate<NpcCar>();
			AddChild(carNode);
			carNode.AddToGroup("MovableObstacles");
			carNode.CarReachedSink += HandleCarReachedSink;
			carNode.Disable();
			_available.Push(carNode);
		}
	}

	public NpcCar Checkout()
	{
		if (_available.Count == 0)
			return null;

		var car = _available.Pop();
		_active.Add(car);
		car.Enable();
		return car;
	}

	public void Return(NpcCar car)
	{
		if (!_active.Contains(car))
			return;

		car.Reset();
		car.Disable();
		_active.Remove(car);
		_available.Push(car);
	}

	public void ReturnAll()
	{
		var carsToReturn = new List<NpcCar>(_active);
		foreach (var car in carsToReturn)
		{
			Return(car);
		}
	}

	private void HandleCarReachedSink(NpcCar car)
	{
		Return(car);
	}
}

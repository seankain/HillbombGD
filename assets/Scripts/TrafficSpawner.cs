using Godot;
using System;
using System.Collections.Generic;

public partial class TrafficSpawner : Node3D
{
	[Export]
	public Path3D[] TrafficPaths;

	[Export]
	public float MinSpawnInterval = 2.0f;

	[Export]
	public float MaxSpawnInterval = 5.0f;

	[Export]
	public float MinCarSpeed = 8.0f;

	[Export]
	public float MaxCarSpeed = 15.0f;

	private TrafficPool _pool;
	private TrafficSimulator _simulator;
	private TrafficLightController _lightController;
	private List<NpcCar> _activeCars;
	private float[] _spawnTimers;
	private RandomNumberGenerator _rng;

	public void Initialize(TrafficPool pool, TrafficSimulator simulator = null)
	{
		_pool = pool;
		_simulator = simulator;
		_activeCars = new List<NpcCar>();
		_rng = new RandomNumberGenerator();
		_rng.Randomize();

		if (TrafficPaths != null)
		{
			_spawnTimers = new float[TrafficPaths.Length];
			for (int i = 0; i < _spawnTimers.Length; i++)
			{
				_spawnTimers[i] = _rng.RandfRange(MinSpawnInterval, MaxSpawnInterval);
			}
		}
	}

	public void SetLightController(TrafficLightController controller)
	{
		_lightController = controller;
	}

	public void SpawnTick(double delta)
	{
		if (_pool == null || TrafficPaths == null)
		{
			return;
		}

		for (int i = 0; i < TrafficPaths.Length; i++)
		{
			if (TrafficPaths[i] == null)
			{
				GD.PrintErr("No traffic paths exist for traffic spawner");
				continue;
			}

			_spawnTimers[i] -= (float)delta;
			if (_spawnTimers[i] <= 0f)
			{
				_spawnTimers[i] = _rng.RandfRange(MinSpawnInterval, MaxSpawnInterval);

				if (_simulator != null && !_simulator.CanSpawnOnPath(TrafficPaths[i], 0f))
					continue;

				if (_lightController != null)
				{
					var lightState = _lightController.GetStateForPath(TrafficPaths[i]);
					if (lightState == TrafficLightState.Red)
						continue;
				}

				var car = _pool.Checkout();
				if (car == null)
				{
					continue;
				}

				var chunk = GetParent<HillChunk>();
				float speed = _rng.RandfRange(MinCarSpeed, MaxCarSpeed);
				car.Activate(TrafficPaths[i], chunk, speed);

				if (_lightController != null)
					car.SetLightController(_lightController);

				_activeCars.Add(car);
			}
		}
	}

	public void ReturnAllCars()
	{
		if (_pool == null || _activeCars == null)
			return;

		foreach (var car in _activeCars)
		{
			_pool.Return(car);
		}
		_activeCars.Clear();
	}
}

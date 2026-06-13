using Godot;
using System.Collections.Generic;

public partial class TrafficSpawner : Node3D
{
	[Export]
	public TrafficWaypoint[] SpawnPoints;

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

		if (SpawnPoints != null)
		{
			_spawnTimers = new float[SpawnPoints.Length];
			for (int i = 0; i < _spawnTimers.Length; i++)
			{
				_spawnTimers[i] = _rng.RandfRange(MinSpawnInterval, MaxSpawnInterval);
			}
		}
	}

	public void SpawnTick(double delta)
	{
		if (_pool == null || SpawnPoints == null)
			return;

		for (int i = 0; i < SpawnPoints.Length; i++)
		{
			if (SpawnPoints[i] == null)
				continue;

			_spawnTimers[i] -= (float)delta;
			if (_spawnTimers[i] <= 0f)
			{
				_spawnTimers[i] = _rng.RandfRange(MinSpawnInterval, MaxSpawnInterval);

				var wp = SpawnPoints[i];

				if (_simulator != null && !_simulator.CanSpawnAt(wp.GlobalPosition))
					continue;

				if (wp.IsStopLine && wp.StopLight != null &&
					wp.StopLight.State == TrafficLightState.Red)
					continue;

				var car = _pool.Checkout();
				if (car == null)
					continue;

				var chunk = GetParent<HillChunk>();
				float speed = _rng.RandfRange(MinCarSpeed, MaxCarSpeed);
				car.Activate(wp, chunk, speed);
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

"""Simulation extension point; dynamic integration is outside milestone 1."""


class Simulator:
    def step(self, *args, **kwargs):
        raise NotImplementedError("dynamic simulation is deferred beyond milestone 1")


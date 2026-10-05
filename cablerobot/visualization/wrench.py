"""Visualization of selected generalized-force coordinates."""


def plot_force_set(force_set, coordinates=(0, 1), *, ax=None):
    import matplotlib.pyplot as plt

    if len(coordinates) != 2:
        raise ValueError("select two generalized-force coordinates")
    if ax is None:
        _, ax = plt.subplots()
    vertices = force_set.vertices
    if vertices.size:
        ax.scatter(vertices[:, coordinates[0]], vertices[:, coordinates[1]], s=10)
    ax.set(xlabel=f"generalized force {coordinates[0]}", ylabel=f"generalized force {coordinates[1]}")
    return ax

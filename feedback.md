* **Be sure that the argument from arcsin is between -1 and 1**

```python
alpha_1 = np.arcsin(
    np.clip(
        2 * np.sin(gamma / 2) /
        dc * (l0 - np.tan((np.pi - gamma) / 2)) *
        (lio / 2 + s0),
        -1, 1
    )
)
```

* **Test different initial variables**
* **Add a visualization to better understand the solution (take example from the paper, but have in mind anastemosis case)**
* **Review wich solvers can be used and its advantages and disadvantages (use the one that fits better to this problem)**
* **Use Relative Paths** : Replace absolute paths with `pathlib` for portability:

```python
   from pathlib import Path

   BASE_DIR = Path(__file__).resolve().parent
   ASSETS_DIR = BASE_DIR / "assets"

   STL_FILE = ASSETS_DIR / "ago_66mm.stl"
   VESSEL_OBJ = ASSETS_DIR / "Arteria_piena.obj"
   VESSEL_VTK = ASSETS_DIR / "Arteria_piena.vtk"
```

* **Verify File Existence** :

```python
   for file in [STL_FILE, VESSEL_OBJ, VESSEL_VTK]:
       if not file.exists():
           raise FileNotFoundError(f"Missing file: {file}")
```

1. **Organize Assets** : Place all STL and OBJ files in an `assets/` folder in your repository for clarity.
2. **Document Structure** : Update your `README.md` to explain the directory layout and asset requirements.

import re
import sys
from pathlib import Path

import pandas as pd
import matplotlib.pyplot as plt

LINE_RE = re.compile(
    r'^(?P<path>[^:]+):#\s*=>\s*Total\s*=\s*(?P<ms>[0-9.]+)\s*ms\s*\[~(?P<fps>[0-9.]+)\s*FPS\]\s*$'
)

THREAD_RE = re.compile(r'-t(?P<t>\d+)\.txt$')

def parse_results_txt(txt_path: Path) -> pd.DataFrame:
    rows = []
    for line in txt_path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line:
            continue
        m = LINE_RE.match(line)
        if not m:
            # ignore silently or raise; here we ignore non-matching lines
            continue

        path = m.group("path")
        ms = float(m.group("ms"))
        fps = float(m.group("fps"))

        # name = filename without extension
        filename = Path(path).name
        name = filename[:-4] if filename.endswith(".txt") else filename

        # threads if present like "...-t16.txt"
        mt = THREAD_RE.search(filename)
        threads = int(mt.group("t")) if mt else None

        # group heuristics (edit if you add more naming conventions)
        group = "other"
        if "baseline" in path:
            group = "baseline"
        elif "/scaling/" in path:
            group = "scaling"
        elif "/simd/" in path:
            group = "simd"
        elif "/omp/" in path:
            group = "omp"
        elif "/sd/" in path:
            group = "sd"
        elif "/morpho/" in path:
            group = "morpho"

        rows.append({
            "path": path,
            "name": name,
            "group": group,
            "threads": threads,
            "ms_total": ms,
            "fps": fps,
        })

    df = pd.DataFrame(rows)
    if df.empty:
        raise RuntimeError("Aucune ligne parsée. Vérifie le format de results.txt.")
    return df

def save_csv(df: pd.DataFrame, out_csv: Path):
    df.sort_values(["group", "name"]).to_csv(out_csv, index=False)

def plot_scaling(df: pd.DataFrame, prefix: str, outdir: Path):
    """
    prefix examples:
      - 'omp-sd'     -> files like omp-sd-t1.txt ...
      - 'omp-morpho' -> files like omp-morpho-t1.txt ...
    """
    d = df[df["name"].str.startswith(prefix) & df["threads"].notna()].copy()
    if d.empty:
        print(f"[WARN] pas de données scaling pour prefix={prefix}")
        return

    d = d.sort_values("threads")
    # baseline for speedup = threads=1
    d1 = d[d["threads"] == 1]
    if d1.empty:
        print(f"[WARN] pas de t1 pour calcul speedup ({prefix})")
        return
    fps1 = float(d1.iloc[0]["fps"])
    d["speedup"] = d["fps"] / fps1
    d["efficiency"] = d["speedup"] / d["threads"]

    # FPS vs threads
    plt.figure()
    plt.plot(d["threads"], d["fps"], marker="o")
    plt.xlabel("Nombre de threads")
    plt.ylabel("FPS")
    plt.title(f"Scalabilité (FPS) - {prefix}")
    plt.grid(True)
    plt.xticks(d["threads"])
    plt.tight_layout()
    plt.savefig(outdir / f"scaling_fps_{prefix}.png", dpi=200)

    # Speedup vs threads
    plt.figure()
    plt.plot(d["threads"], d["speedup"], marker="o", label="Speedup mesuré")
    plt.plot(d["threads"], d["threads"], linestyle="--", label="Idéal (linéaire)")
    plt.xlabel("Nombre de threads")
    plt.ylabel("Speedup (FPS / FPS@t1)")
    plt.title(f"Scalabilité (Speedup) - {prefix}")
    plt.grid(True)
    plt.xticks(d["threads"])
    plt.legend()
    plt.tight_layout()
    plt.savefig(outdir / f"scaling_speedup_{prefix}.png", dpi=200)

    # Efficiency vs threads
    plt.figure()
    plt.plot(d["threads"], d["efficiency"], marker="o")
    plt.xlabel("Nombre de threads")
    plt.ylabel("Efficacité (speedup / threads)")
    plt.title(f"Efficacité parallèle - {prefix}")
    plt.grid(True)
    plt.xticks(d["threads"])
    plt.tight_layout()
    plt.savefig(outdir / f"scaling_efficiency_{prefix}.png", dpi=200)

def plot_versions_bar(df: pd.DataFrame, outdir: Path):
    """
    Compare quelques versions importantes: baseline, omp, simd...
    On prend tout ce qui n'a pas threads (donc pas scaling).
    """
    d = df[df["threads"].isna()].copy()

    # On garde une sélection lisible (tu peux ajouter/enlever ici)
    keep_order = [
        "baseline",     # results/baseline/baseline.txt
        "motion2",      # results/baseline/motion2.txt (baseline motion2)
        "omp_v1",       # results/omp/omp_v1.txt
        "omp-cca",
        "omp-ccl",
        "sd-coll2",
        "sd-rowptr",
        "sd-rowptr-omp",
        "sd-simd",
        "morpho-simd",
        "morpho-rowptr",
        "morpho-rowptr_cca",
        "test",         # results/temp/test.txt
    ]

    d["short"] = d["name"]
    dsel = d[d["short"].isin(keep_order)].copy()
    if dsel.empty:
        print("[WARN] Pas de versions trouvées pour le bar chart (adapte keep_order).")
        return

    dsel["short"] = pd.Categorical(dsel["short"], categories=keep_order, ordered=True)
    dsel = dsel.sort_values("short")

    plt.figure(figsize=(10, 4.5))
    plt.bar(dsel["short"].astype(str), dsel["fps"])
    plt.xticks(rotation=45, ha="right")
    plt.ylabel("FPS")
    plt.title("Évolution du throughput selon les versions")
    plt.grid(True, axis="y")
    plt.tight_layout()
    plt.savefig(outdir / "versions_fps.png", dpi=200)

def main():
    if len(sys.argv) < 2:
        print("Usage: python plot_results.py results.txt [outdir]")
        sys.exit(1)

    txt_path = Path(sys.argv[1])
    outdir = Path(sys.argv[2]) if len(sys.argv) >= 3 else Path("plots")
    outdir.mkdir(parents=True, exist_ok=True)

    df = parse_results_txt(txt_path)
    save_csv(df, outdir / "results_parsed.csv")

    # Courbes scaling principales
    plot_scaling(df, "omp-sd", outdir)
    plot_scaling(df, "omp-morpho", outdir)

    # Graphe d'évolution des versions
    plot_versions_bar(df, outdir)

    print(f"OK. CSV + figures dans: {outdir.resolve()}")

if __name__ == "__main__":
    main()

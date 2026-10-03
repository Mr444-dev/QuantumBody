# QuantumBody

**Niewielki, niezależny od bibliotek silnik fizyki brył sztywnych 3D w nowoczesnym C++.** To prototyp edukacyjny z ustalonym krokiem symulacji, kolizjami kul i obróconych pudeł, iteracyjnym solverem impulsowym, powtarzalnymi scenami oraz prostym odtwarzaczem przebiegów.

> **Stan:** prototyp, podstawa milestone’ów M1–M4. Działają ścieżka fizyki CPU, sceny CLI, powtarzalny zapis przebiegu, pomiary czasu i testy kontrolne. CUDA, płyny, tkanina, przeguby, CCD i pełny renderer czasu rzeczywistego pozostają kolejnymi etapami. Wyniki benchmarków zależą od komputera i nie są deklarowane z góry.

## Działające elementy

- Biblioteka fizyki w C++20 bez zewnętrznych zależności uruchomieniowych.
- Integracja semi-implicit Euler dla ruchu liniowego i orientacji reprezentowanej kwaternionem.
- Kule i obrócone pudełka; testy sphere-sphere, sphere-OBB oraz OBB-OBB (SAT na 15 osiach).
- Faza broad-phase sweep-and-prune z konserwatywnymi AABB w przestrzeni świata.
- Iteracyjne impulsy kontaktowe, tarcie Coulomba, odpowiedź kątowa i korekta penetracji.
- Powtarzalne sceny `stack`, `swarm`, `drop` i stress test `failure-lab`.
- Eksport trajektorii JSON, metryki każdego kroku CSV i izometryczny odtwarzacz przebiegu.
- Testy CTest: swobodny spadek, powtarzalność, kontakt z podłożem i odrzucanie błędnego kroku.
- GitHub Actions dla Windows i Ubuntu.

## Kompilacja

Wymagania: CMake 3.21+ oraz kompilator C++20 (MSVC 2022, GCC 11+ lub Clang 14+).

```powershell
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

## Uruchamianie

```powershell
.\build\Release\quantumbody.exe --scene stack --steps 600
.\build\Release\quantumbody.exe --scene swarm --count 500 --seed 42 --steps 900
.\build\Release\quantumbody.exe --scene drop --csv work\drop.csv
```

Program wypisuje jeden rekord JSON z metrykami. Opcja `--csv FILE` zapisuje czas kroku, liczbę kandydatów broad-phase i liczbę kontaktów.

## Odtwarzacz

```powershell
.\build\Release\quantumbody.exe --scene stack --steps 600 --record-every 3 --trajectory web\trajectory.json
python -m http.server 8000
```

Otwórz `http://localhost:8000/web/` i wczytaj `trajectory.json`. Podgląd nie wymaga bibliotek JavaScript.

## Zakres i ograniczenia

Architekturę opisuje [docs/architecture.md](docs/architecture.md), a model matematyczny [docs/physics.md](docs/physics.md). Para brył dostaje obecnie pojedynczy punkt kontaktu wyznaczony z punktów podparcia. To upraszcza implementację, ale daje słabsze stosy pudeł niż wielopunktowy manifold z clippingiem. Sweep-and-prune może zwalniać, gdy wiele brył mocno na siebie nachodzi. Dostępne są kule i pudełka; CCD, przeguby, usypianie obiektów, SIMD, obliczenia GPU, płyny, tkanina i podgląd na żywo nie są jeszcze zaimplementowane.

## Następne etapy

1. Manifoldy wielu kontaktów i rozszerzone testy regresji.
2. Trwałe kontakty, więzy odległościowe/przegubowe i scena łańcucha.
3. Tkanina XPBD oraz płyn SPH z siatką sąsiedztwa.
4. Opcjonalny backend CUDA z porównaniem wyników CPU/GPU.
5. Interaktywny renderer debugowania i profiler.
6. CCD, testy skrajnych przypadków, sanitizery i powtarzalna metodologia benchmarków.

## Licencja

MIT — szczegóły w pliku [LICENSE](LICENSE).

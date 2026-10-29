# Adaptive 9D Boid Sound Engine — Technical Specification

## 01. Overview

The Adaptive 9D Boid Sound Engine is a swarm-driven modal synthesis system in which a population of agents (boids) is represented by a 9-dimensional feature vector. The system does not simulate physical forces; instead, it constructs a structured feature space that is continuously mapped into:

- 3D spatial field (for sound placement)
- density field (for excitation and spectral control)
- temporal excitation field (trigger system)
- modal synthesis layer (resonant sound generation)

Sound is generated through independent resonant filters (modal bank) rather than physical coupling systems. Interaction between agents emerges implicitly through shared statistical fields such as inter-boid distance, local density, and global spatial normalization.

The system combines:

- high-dimensional feature aggregation
- toroidal spatial projection
- event-driven excitation
- modal resonator synthesis
- higher-order spatial audio rendering

Each boid acts as an independent sound source whose spectral and temporal behavior continuously adapts to the collective spatial structure of the swarm.

---

## 02. 9D State Representation

Each boid consists of:

- `position[0..8]` (9D feature vector)
- `velocity[0..8]` (9D feature vector)

The 9D state space is partitioned into three independent 3D subspaces:

```text
| Subspace | Dimensions | Interpretation                      |
| -------- | ---------- | ----------------------------------- |
| S1       | 0–2        | Primary spatial feature vector      |
| S2       | 3–5        | Secondary structural feature vector |
| S3       | 6–8        | Tertiary structural feature vector  |
```
These subspaces are not orthogonalized and are not treated as geometric basis vectors in a strict mathematical sense. Instead, they function as parallel feature channels that are later combined through weighted linear fusion during spatial reconstruction.

The resulting 3D representation is therefore not a projection of a true geometric 9D space, but a feature-driven synthesis of three independent behavioral domains.

---

## 03. Spatial Aggregation Model

### 3.1 Subspace Fusion

Each boid’s spatial position is computed as a weighted sum of its three 3D subspaces:

```text
spatialPos[i] =
    w1 * pos[0..2] +
    w2 * pos[3..5] +
    w3 * pos[6..8]
```

The weights are normalized per system configuration so that:

```text
w1 + w2 + w3 = 1
```

This fusion step defines the primary mapping from the 9D feature space into a 3D spatial field used for all downstream audio processes.

**3.2 Temporal Smoothing**

Spatial positions are low-pass filtered:

```text
spatialPos(t) = α * spatialPos(t-1) + (1 - α) * fusedPosition
```

This temporal smoothing stabilizes the per-boid spatial field and reduces high-frequency jitter in derived synthesis parameters, particularly frequency mapping, density estimation, and spatialization inputs.

### 03.3 Dynamic Spatial Normalization (Swarm Frame)

In addition to temporal smoothing, the spatial system performs a global normalization step based on the swarm’s evolving spatial extent.

At runtime, the system computes:
- the bounding box of all boid positions
- the swarm centroid
- a global scale derived from spatial extent

These values are then low-pass filtered over time, producing a slowly evolving affine transformation of the spatial field.

This means the spatial coordinate system is not fixed, but continuously adapts to the swarm itself.

Consequences:
- global expansion compresses perceived spatial relationships
- global contraction increases spatial resolution
- frequency mapping, density, and trigger behavior are implicitly influenced by this evolving scale

---

### 04. Torus Wrapping Model

All spatial computations operate in a periodic domain:

- positions are wrapped into a toroidal space of size 2 units
- differences are wrapped to ensure shortest-path consistency

```text
p -= round(p / 2) * 2;
```

This ensures that all spatial relationships are computed on a continuous torus rather than in unbounded Euclidean space, preventing boundary discontinuities and maintaining consistent neighborhood relations across wrap boundaries.

This wrapping model applies to:

- position representation
- velocity reconstruction (via wrapped position differences)
- inter-boid distance calculations
- normalization-dependent spatial statistics

---

### 05. Density Model

Each boid’s density is computed from its average wrapped distance to all other boids:

```text
meanDist[i] = average distance to all j ≠ i
density[i]  = exp(-meanDist[i] * DENSITY_EXP_SCALE)
```
Distances are computed using torus-wrapped spatial differences, ensuring continuity across periodic boundaries.

Properties:

- density is defined locally per boid
- there is no explicit global density field in the system
- a separate global mean distance is computed independently and used for normalization and comparative scaling

Interpretation:

- dense clustering → higher density values
- sparse distribution → lower density values

Density acts as a control modulation signal rather than a spatial field, influencing synthesis parameters such as frequency weighting, excitation behavior, and spectral shaping.

---

### 06. Velocity Model

Velocity is reconstructed using torus-consistent differences between consecutive spatial frames:

```text
velocity[i] = wrap(spatialPos[i] - previousSpatialPos[i])
```

This ensures that velocity remains continuous across torus boundaries and does not produce artificial spikes when a boid crosses a wrap edge.

Velocity is used as a secondary excitation descriptor and influences the synthesis process through:

- spectral instability (frequency modulation jitter)
- amplitude variation across modal components
- mode-level modulation (especially higher-order modes)

Velocity plays a secondary role in the system and does not directly influence trigger rate or excitation density. Its primary function is subtle spectral modulation rather than structural control of system dynamics.

---

### 07. Trigger System (Event Excitation)

Each boid maintains an independent deterministic accumulator that drives event-based excitation of the modal synthesis layer:

```text
acc[i] += triggerRate[i] / sampleRate
if acc[i] ≥ 1:
    excite modal bank
    acc[i] -= 1
```
This produces a deterministic, continuous-time event system based on phase accumulation. Although it can exhibit irregular emergent timing due to spatial modulation of rates, it is not stochastic in nature.

**Trigger rate computation:**

The trigger rate is derived from spatial structure using both relative and absolute distance measures:

```text
adaptive  = meanDist[i] / globalMeanDist
absolute  = meanDist[i] / ABSOLUTE_DISTANCE_SCALE

relative  = mix * adaptive + (1 - mix) * absolute
compression = clamp(1 - relative, 0, 1)

triggerRate = minRate + (maxRate - minRate) * compression^exponent
```

**Key property:**

Triggering is driven by relative structure, not absolute density.

---

### 08 Modal Synthesis System

Each boid controls an independent modal bank:

- N modes per boid (default: 8)
- each mode is a resonant filter

**Mode parameterization:**

For mode index m:

```text
freq[m] = baseFreq * (1 + spread * m + jitter)
bw[m]   = baseBW * (1 + 0.3 * m)
amp[m]  = baseAmp / (1 + decay * m)
```

where:

- baseFreq is derived from spatial position + density
- jitter is a nonlinear term driven by velocity and mode index

```text
jitter = 0.02 * (1 + energy) * sin(m)
```

**Interpretation:**

- higher modes → broader, weaker spectral components
- velocity increases spectral instability
- density increases spectral brightness

### 09. Modal Excitation Model

Each trigger event injects energy into all modes of a boid:

```text
excitation[i][m] += triggerEnergy
```

Energy is reset after each audio frame.

Modal output is summed per boid:

```text
boidOutput[i] = Σ modeOutput[i][m] * weight[m]
```

Note: excitation is applied at the boid level. Individual modes are not independently triggered; instead, the modal bank distributes excitation internally across its resonant modes.

---

### 10. Spatial Audio Encoding

**Encoding:**

```text
azimuth = atan2(x, z)
distance = clamp(|position|)

gain = 1 / (1 + k * distance)
```

The signal is encoded into a 3rd-order ambisonic field (7 channels):

- W (omnidirectional)
- X/Y/Z (first order)
- higher-order components (2nd + 3rd order)

The ambisonic representation is fixed to 3rd order (7 channels) and does not dynamically adapt its order or normalization based on scene complexity.

---

### 11. Ambisonic Decoding

The ambisonic field is decoded into stereo:

```text
stereo = decode(ambiField, speakerAngle)
```

Two virtual speaker positions are used for left/right output.

---

### 12. Output Stage

Final output is saturated using soft clipping:

```text
output = tanh(signal * drive)
```

This prevents modal summation from exceeding stable amplitude bounds.

---

### 13. System Architecture

```text
[OSC Input]
     ↓
[Boid State: 9D vectors]
     ↓
[Subspace Fusion (S1 + S2 + S3)]
     ↓
[Spatial Field (3D smoothed torus space)]
     ↓
 ┌──────────────────────────────┐
 │ Density Computation          │
 │ Velocity Reconstruction      │
 │ Distance Ratios             │
 └──────────────────────────────┘
     ↓
[Trigger System (event-driven excitation)]
     ↓
[Modal Bank (per boid, multi-mode resonators)]
     ↓
[Audio Summation per Boid]
     ↓
[Ambisonic Encoding (3rd order)]
     ↓
[Ambisonic Decoding]
     ↓
[Stereo Output + tanh saturation]
```
 
---

### 14. System Characteristics

**Emergent behavior sources:**

- ratio-based trigger system (relative geometry)
- nonlinear modal jitter term
- density-dependent excitation scaling
- torus-wrapped spatial continuity
- multi-mode spectral decay hierarchy 

**Not present in implementation:**

- no physical force simulation
- no orthogonal 9D basis construction
- no explicit coupling matrix between boids
- no true geometric decomposition into r/t1/t2 vectors

---

### 15. Conceptual Summary

The system is best described as:

The system is a torus-bound, statistically coupled swarm synthesis architecture in which 9D agent states are projected into a dynamically normalized 3D feature space. Spatial geometry, density structure, and torus-consistent motion jointly control a distributed modal resonator network. Sound emerges through event-driven excitation, perceptual frequency mapping, and continuously evolving spectral topology rather than explicit physical simulation.

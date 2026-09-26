using System.Numerics;

namespace Scanner.Core.Photogrammetry;

/// <summary>A corner found in one photo: pixel coordinates of that photo (x right, y down, pixel centres at
/// integers), sub-pixel where the detector refines them, and the detector's response.</summary>
public readonly record struct Feature(float X, float Y, float Response);

/// <summary>A feature of one photo matched to a feature of another: photo indices and indices into each photo's
/// feature list, with the matching score (normalized cross-correlation).</summary>
public readonly record struct FeatureMatch(int ViewA, int FeatureA, int ViewB, int FeatureB, float Score);

/// <summary>Where a 3D point was seen: the photo index and the pixel.</summary>
public readonly record struct Observation(int View, float X, float Y);

/// <summary>A 3D point (world frame, metres) and the photos it was seen in.</summary>
public sealed record Track(Vector3 Point, Observation[] Observations);

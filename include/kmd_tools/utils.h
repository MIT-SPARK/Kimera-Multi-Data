/**
 * @file utils.h
 * @brief Miscellaneous utility functions
 * @author Parker Lusk <plusk@mit.edu>
 * @date 15 April 2023
 */

#pragma once

#include <map>
#include <filesystem>
#include <string>
#include <unordered_map>

#include <Eigen/Geometry>

namespace kmd_tools {
namespace utils {

/**
 * @brief      Finds the value with the closest key to the desired key.
 *
 * @param[in]  map   The map to search in
 * @param[in]  key   The desired key
 *
 * @tparam     T1    Key type
 * @tparam     T2    Value type
 *
 * @return     ConstIterator to the closest element in the map
 */
template <typename T1, typename T2>
typename std::map<T1, T2>::const_iterator find_closest(const std::map<T1, T2>& map, T1 key)
{
  auto lower_bound = map.lower_bound(key);
  if (lower_bound == map.end()) return --lower_bound;
  auto upper_bound = lower_bound; upper_bound++;
  if (upper_bound == map.end()) return lower_bound;
  auto dist_to_lower = lower_bound->first - key;
  auto dist_to_upper = upper_bound->first - key;
  return (dist_to_upper < dist_to_lower) ? upper_bound : lower_bound;
}

/**
 * @brief      Linearly interpolate between two poses.
 *
 * @param[in]  t     Desired time such that ta < t < tb
 * @param[in]  ta    Time of first pose
 * @param[in]  Ta    First pose
 * @param[in]  tb    Time of second pose
 * @param[in]  Tb    Second pose
 *
 * @tparam     T     Type used for time
 *
 * @return     Interpolant, i.e., pose at desired time t.
 */
template <typename T>
Eigen::Affine3d interpolate(T t, T ta, const Eigen::Affine3d& Ta,
                            T tb, const Eigen::Affine3d& Tb)
{
  // assert(tb >= ta);
  // assert(ta <= t && t <= tb);
  const double alpha = (ta == tb) ? 0 : (static_cast<double>(t - ta) / (tb - ta));

  const Eigen::Quaterniond qa(Ta.linear());
  const Eigen::Vector3d pa = Ta.translation();
  const Eigen::Quaterniond qb(Tb.linear());
  const Eigen::Vector3d pb = Tb.translation();

  Eigen::Affine3d Tinterp;
  Tinterp.linear() = qa.slerp(alpha, qb).toRotationMatrix();
  Tinterp.translation() = (1 - alpha) * pa + alpha * pb;

  return Tinterp;
}

/**
 * @brief      Given a map of poses, keyed by time, interpolate between
 *             two of the poses to estimate the pose at desired time t.
 *
 * @param[in]  map   Map of poses, keyed by time
 * @param[in]  t     The time to retrieve a pose at
 *
 * @tparam     T     Type used for time
 *
 * @return     Interpolant, i.e., pose at desired time t
 */
template <typename T>
Eigen::Affine3d find_interpolant(const std::map<T, Eigen::Affine3d>& map, T t)
{
  // assert(!map.empty());

  const auto b = map.upper_bound(t);
  const auto a = std::prev(b);

  if (b == map.begin()) return b->second; // if before any times, use first
  if (b == map.end()) return a->second; // if after, use last
  if (t == a->first) return a->second; // early return

  return interpolate(t, a->first, a->second, b->first, b->second);
}

} // utils
} // ns kmd_tools

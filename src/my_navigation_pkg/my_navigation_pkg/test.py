def control_loop(self):
        if not self.ranges:
            return
        twist = Twist()
        # Extract right-side and front distances
        ranges = self.ranges
        right_distance = min(ranges[270:360])  # Approx 270° to 360° (right side)
        front_distance = min(ranges[0:30] + ranges[330:360])  # Approx 0° to 30° and 330° to 360° (front)

        # Obstacle avoidance
        if front_distance < self.forward_threshold:
            self.get_logger().info("Obstacle ahead! Turning left.")
            twist.linear.x = 0.0
            twist.angular.z = self.angular_speed
        else:
            # Wall following behavior
            if right_distance > self.right_distance_target + 0.1:  # Too far from the wall
                self.get_logger().info("Too far from wall, turning right.")
                twist.linear.x = self.linear_speed
                twist.angular.z = -self.angular_speed
            elif right_distance < self.right_distance_target - 0.1:  # Too close to the wall
                self.get_logger().info("Too close to wall, turning left.")
                twist.linear.x = self.linear_speed
                twist.angular.z = self.angular_speed
            else:  # Maintain the wall-following distance
                self.get_logger().info("Following the wall.")
                twist.linear.x = self.linear_speed
                twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)


def find_nearest_left_wall(self):
        """Find the nearest point to the left wall."""
        if self.ranges is None:
            return None, float('inf')

        left_start_idx = int(len(self.ranges) * 3 / 4)  # Start of the left side (270 degrees)
        left_end_idx = len(self.ranges) - 1            # End of the left side (360 degrees)

        min_distance = float('inf')
        min_idx = None
        for i in range(left_start_idx, left_end_idx + 1):
            distance = self.ranges[i]
            if not math.isinf(distance) and not math.isnan(distance) and distance < min_distance:
                min_distance = distance
                min_idx = i

        return min_idx, min_distance
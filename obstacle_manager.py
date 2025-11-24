"""
障礙物管理器模組 - 智能網格線分段版
實現方案一：網格線智能分段避障，最大化偵察覆蓋率
"""

import math
from typing import List, Tuple, Optional
from logger_utils import logger

try:
    from config import Config
except ImportError:
    class Config:
        EARTH_RADIUS_M = 6378137.0

class Obstacle:
    """障礙物資料類"""
    def __init__(self, center: Tuple[float, float], radius: float, safe_distance: float = 1.0):
        self.center = center  # (lat, lon)
        self.radius = radius  # 公尺
        self.safe_distance = safe_distance  # 安全距離（公尺）
        self.marker = None  # 地圖標記
        self.circle = None  # 圓形顯示
        self.safe_circle = None # 安全範圍顯示
        
    @property
    def effective_radius(self):
        """有效半徑 = 障礙物半徑 + 安全距離"""
        return self.radius + self.safe_distance


class ObstacleManager:
    """
    障礙物管理器 - 智能網格線分段版
    
    核心功能：
    1. 識別網格掃描線結構
    2. 檢測掃描線是否穿過障礙物
    3. 將穿過障礙物的掃描線分段
    4. 生成繞行路徑，保持最大偵察覆蓋率
    """
    
    def __init__(self):
        self.obstacles: List[Obstacle] = []
        self.earth_radius_m = 111111.0  # 每度約111111公尺
        
    def add_obstacle(self, center: Tuple[float, float], radius: float, 
                    safe_distance: float = 1.0) -> Obstacle:
        """添加障礙物"""
        obstacle = Obstacle(center, radius, safe_distance)
        self.obstacles.append(obstacle)
        logger.info(f"添加障礙物：中心{center}, 半徑{radius}m, 安全距離{safe_distance}m")
        return obstacle
    
    def remove_obstacle(self, obstacle: Obstacle):
        """移除障礙物"""
        if obstacle in self.obstacles:
            self.obstacles.remove(obstacle)
            logger.info(f"移除障礙物：{obstacle.center}")
            return True
        return False

    def remove_nearest_obstacle(self, coords: Tuple[float, float], threshold_m: float = 50.0):
        """移除指定座標最近的障礙物"""
        if not self.obstacles:
            return None
            
        nearest_obs = None
        min_dist = float('inf')
        
        for obs in self.obstacles:
            dist = self.calculate_distance(coords, obs.center)
            if dist < min_dist:
                min_dist = dist
                nearest_obs = obs
                
        if nearest_obs:
            self.remove_obstacle(nearest_obs)
            return nearest_obs
        return None
    
    def clear_all(self):
        """清除所有障礙物"""
        self.obstacles.clear()
        logger.info("清除所有障礙物")

    def filter_waypoints_with_detour(self, waypoints: List[Tuple[float, float]], 
                                     boundary_corners: List[Tuple[float, float]] = None) -> List[Tuple[float, float]]:
        """
        智能網格線分段避障演算法（方案一）
        
        流程：
        1. 識別掃描線段（通過距離分析）
        2. 檢測掃描線是否穿過障礙物
        3. 如果穿過，將線段智能分段：前段 + 繞行段 + 後段
        4. 保持最大偵察覆蓋率
        """
        if not waypoints or not self.obstacles:
            return waypoints

        # 識別掃描線段結構
        scan_segments = self._identify_scan_segments(waypoints)
        
        result_waypoints = []
        processed_indices = set()
        
        for seg_type, indices in scan_segments:
            if seg_type == "scan":
                # 這是一條掃描線段
                start_idx, end_idx = indices
                p1 = waypoints[start_idx]
                p2 = waypoints[end_idx]
                
                # 檢查是否穿過障礙物
                colliding_obstacles = self.check_segment_collision(p1, p2)
                
                if not colliding_obstacles:
                    # 無障礙物，正常添加掃描線兩端點
                    if start_idx not in processed_indices:
                        result_waypoints.append(p1)
                        processed_indices.add(start_idx)
                    if end_idx not in processed_indices:
                        result_waypoints.append(p2)
                        processed_indices.add(end_idx)
                else:
                    # 有障礙物，智能分段處理
                    segmented_waypoints = self._segment_scan_line(p1, p2, colliding_obstacles, boundary_corners)
                    
                    # 添加分段後的航點（去重）
                    for wp in segmented_waypoints:
                        if wp not in result_waypoints:
                            result_waypoints.append(wp)
                    
                    processed_indices.add(start_idx)
                    processed_indices.add(end_idx)
                    logger.info(f"掃描線 {start_idx}-{end_idx} 穿過障礙物，已分段處理，生成{len(segmented_waypoints)}個航點")
            
            elif seg_type == "turn":
                # 轉向段，直接添加未處理的點
                for idx in indices:
                    if idx not in processed_indices:
                        result_waypoints.append(waypoints[idx])
                        processed_indices.add(idx)
        
        # 確保所有未處理的點都被添加
        for i, wp in enumerate(waypoints):
            if i not in processed_indices:
                result_waypoints.append(wp)
                processed_indices.add(i)
        
        return result_waypoints
    
    def _identify_scan_segments(self, waypoints: List[Tuple[float, float]]) -> List[Tuple[str, Tuple[int, int]]]:
        """
        識別掃描線段結構
        
        基於距離分析：
        - 長距離段（>閾值）：掃描線
        - 短距離段（<閾值）：轉向段
        
        返回: [(segment_type, (start_index, end_index)), ...]
        segment_type: "scan" 或 "turn"
        """
        if len(waypoints) < 2:
            return []
        
        segments = []
        
        # 計算所有相鄰點之間的距離
        distances = []
        for i in range(len(waypoints) - 1):
            dist = self.calculate_distance(waypoints[i], waypoints[i + 1])
            distances.append((i, i + 1, dist))
        
        if not distances:
            return []
        
        # 找出距離中位數作為閾值
        sorted_dists = sorted([d[2] for d in distances])
        median_dist = sorted_dists[len(sorted_dists) // 2]
        max_dist = sorted_dists[-1]
        
        # 閾值：使用最大距離的50%，確保只識別真正的長掃描線
        # 避免將短距離誤判為掃描線
        threshold = max(median_dist * 0.6, max_dist * 0.5)
        
        # 識別掃描線段
        for idx_start, idx_end, dist in distances:
            if dist > threshold:
                segments.append(("scan", (idx_start, idx_end)))
            else:
                segments.append(("turn", (idx_start, idx_end)))
        
        logger.info(f"識別掃描結構：{len([s for s in segments if s[0]=='scan'])}條掃描線，閾值={threshold:.2f}m")
        return segments
    
    def _segment_scan_line(self, p1: Tuple[float, float], p2: Tuple[float, float],
                          obstacles: List[Obstacle],
                          boundary_corners: Optional[List[Tuple[float, float]]]) -> List[Tuple[float, float]]:
        """
        將穿過障礙物的掃描線分段（遞歸處理多個障礙物）

        策略：
        1. 計算掃描線與障礙物的交點
        2. 生成繞行路徑（沿著障礙物安全邊界）
        3. 遞歸處理剩餘的障礙物
        4. 返回: [p1, 繞行點們..., p2]

        返回: 分段後的航點列表
        """
        if not obstacles:
            return [p1, p2]

        # 找到最接近起點的障礙物
        closest_obstacle = None
        min_dist_to_start = float('inf')

        for obs in obstacles:
            # 計算障礙物中心到起點的距離
            dist = self.calculate_distance(p1, obs.center)
            if dist < min_dist_to_start:
                min_dist_to_start = dist
                closest_obstacle = obs

        if closest_obstacle is None:
            return [p1, p2]

        # 計算線段與最近障礙物的交點
        intersection_points = self._calculate_line_circle_intersection(p1, p2, closest_obstacle)

        if len(intersection_points) < 2:
            # 沒有足夠的交點，嘗試處理其他障礙物
            remaining_obstacles = [o for o in obstacles if o != closest_obstacle]
            if remaining_obstacles:
                return self._segment_scan_line(p1, p2, remaining_obstacles, boundary_corners)
            return [p1, p2]

        # 生成繞行路徑（沿著障礙物安全邊界的切線）
        detour_points = self._generate_tangent_detour(p1, p2, closest_obstacle,
                                                      intersection_points[0], intersection_points[1])

        if not detour_points:
            logger.warning(f"繞行點生成失敗，掃描線長度可能太短")
            # 嘗試處理其他障礙物
            remaining_obstacles = [o for o in obstacles if o != closest_obstacle]
            if remaining_obstacles:
                return self._segment_scan_line(p1, p2, remaining_obstacles, boundary_corners)
            return [p1, p2]

        # 驗證繞行點是否在邊界內
        valid_detour = []
        for i, dp in enumerate(detour_points):
            if boundary_corners is None:
                valid_detour.append(dp)
            elif self.point_in_polygon(dp, boundary_corners):
                valid_detour.append(dp)
            else:
                logger.warning(f"繞行點{i+1}: ({dp[0]:.6f}, {dp[1]:.6f}) 超出邊界")
                # 使用備用策略：直接在線段上選點
                if i == 0:  # 進入點：在30%位置
                    fallback = self._interpolate_point(p1, p2, 0.30)
                else:  # 離開點：在70%位置
                    fallback = self._interpolate_point(p1, p2, 0.70)

                if self.point_in_polygon(fallback, boundary_corners):
                    valid_detour.append(fallback)

        if not valid_detour:
            logger.warning(f"無法生成有效繞行路徑")
            # 嘗試處理其他障礙物
            remaining_obstacles = [o for o in obstacles if o != closest_obstacle]
            if remaining_obstacles:
                return self._segment_scan_line(p1, p2, remaining_obstacles, boundary_corners)
            return [p1, p2]

        # 構建當前障礙物的繞行路徑
        current_path = [p1] + valid_detour + [p2]

        # 檢查是否還有其他障礙物需要處理
        remaining_obstacles = [o for o in obstacles if o != closest_obstacle]
        if not remaining_obstacles:
            logger.info(f"成功生成繞行路徑: {len(valid_detour)}個繞行點")
            return current_path

        # 遞歸處理剩餘障礙物：檢查每個線段是否與其他障礙物碰撞
        final_path = [current_path[0]]
        for i in range(len(current_path) - 1):
            seg_start = current_path[i]
            seg_end = current_path[i + 1]

            # 檢查這個線段是否與剩餘障礙物碰撞
            colliding = []
            for obs in remaining_obstacles:
                if self.line_intersects_circle(seg_start, seg_end, obs.center, obs.effective_radius):
                    colliding.append(obs)

            if colliding:
                # 遞歸處理這個線段
                sub_path = self._segment_scan_line(seg_start, seg_end, colliding, boundary_corners)
                # 添加子路徑（排除起點，因為已經在final_path中）
                final_path.extend(sub_path[1:])
            else:
                # 無碰撞，直接添加終點
                final_path.append(seg_end)

        logger.info(f"完整繞行路徑: 處理{len(obstacles)}個障礙物, 生成{len(final_path)}個航點")
        return final_path
    
    def _calculate_line_circle_intersection(self, p1: Tuple[float, float], 
                                           p2: Tuple[float, float],
                                           obstacle: Obstacle) -> List[Tuple[float, float]]:
        """
        計算線段與圓形障礙物的交點
        
        使用參數方程：
        - 線段: P(t) = P1 + t(P2 - P1), t ∈ [0, 1]
        - 圓: (x - cx)² + (y - cy)² = r²
        
        返回: 交點列表（地理座標）
        """
        lat1, lon1 = p1
        lat2, lon2 = p2
        cx, cy = obstacle.center
        radius = obstacle.effective_radius
        
        # 使用與waypoint_generator相同的座標轉換
        cosLat0 = math.cos(math.radians(cx))
        
        def to_meters(lat, lon):
            # 與waypoint_generator.py的project_and_rotate一致
            x = (lon - cy) * self.earth_radius_m * cosLat0
            y = (lat - cx) * self.earth_radius_m
            return x, y
        
        def to_latlon(x, y):
            # 與waypoint_generator.py的rotate_back_to_geographic一致
            lat = y / self.earth_radius_m + cx
            lon = x / (self.earth_radius_m * cosLat0) + cy
            return lat, lon
        
        x1, y1 = to_meters(lat1, lon1)
        x2, y2 = to_meters(lat2, lon2)
        
        # 線段參數方程
        dx = x2 - x1
        dy = y2 - y1
        
        # 代入圓方程，得到二次方程 at² + bt + c = 0
        a = dx * dx + dy * dy
        b = 2 * (dx * x1 + dy * y1)
        c = x1 * x1 + y1 * y1 - radius * radius
        
        discriminant = b * b - 4 * a * c
        
        if discriminant < 0 or a == 0:
            return []  # 無交點
        
        # 計算兩個解
        t1 = (-b - math.sqrt(discriminant)) / (2 * a)
        t2 = (-b + math.sqrt(discriminant)) / (2 * a)
        
        # 篩選在線段上的交點 (0 <= t <= 1)
        intersections = []
        for t in [t1, t2]:
            if 0 <= t <= 1:
                x = x1 + t * dx
                y = y1 + t * dy
                intersections.append(to_latlon(x, y))
        
        return intersections
    
    def _generate_tangent_detour(self, p1: Tuple[float, float], p2: Tuple[float, float],
                                 obstacle: Obstacle,
                                 inter1: Tuple[float, float],
                                 inter2: Tuple[float, float]) -> List[Tuple[float, float]]:
        """
        生成沿著障礙物邊界的規律繞行路徑（擴展版 - 從交點外開始）

        策略：
        1. 在交點之外開始繞行，確保不穿越障礙物
        2. 沿著安全邊界的圓周走
        3. 生成規律的圓弧路徑

        返回: [進入點, 中間點們..., 離開點]
        """
        cx, cy = obstacle.center
        # 安全半徑：在effective_radius基礎上額外增加0.5米作為緩衝
        safe_radius = obstacle.effective_radius + 0.5

        # 座標轉換函數
        cosLat0 = math.cos(math.radians(cx))

        def to_meters(lat, lon):
            x = (lon - cy) * self.earth_radius_m * cosLat0
            y = (lat - cx) * self.earth_radius_m
            return x, y

        def to_latlon(x, y):
            lat = y / self.earth_radius_m + cx
            lon = x / (self.earth_radius_m * cosLat0) + cy
            return lat, lon

        # 轉換到公尺座標系（以障礙物中心為原點）
        x1, y1 = to_meters(p1[0], p1[1])
        x2, y2 = to_meters(p2[0], p2[1])

        # 計算線段方向
        dx = x2 - x1
        dy = y2 - y1
        length = math.sqrt(dx * dx + dy * dy)

        if length < 1.0:
            return []

        # 計算兩個交點對應的角度
        xi1, yi1 = to_meters(inter1[0], inter1[1])
        xi2, yi2 = to_meters(inter2[0], inter2[1])

        angle1 = math.atan2(yi1, xi1)
        angle2 = math.atan2(yi2, xi2)

        # 確定繞行方向（選擇遠離線段的方向）
        # 計算線段中點到圓心的向量
        mid_x = (x1 + x2) / 2
        mid_y = (y1 + y2) / 2

        # 線段方向的垂直向量
        perp_x = -dy / length
        perp_y = dx / length

        # 判斷應該走哪一側的圓弧（選擇遠離線段的那一側）
        # 測試點：圓心向外的方向
        dot_product = perp_x * (-mid_x) + perp_y * (-mid_y)

        if dot_product > 0:
            # 需要確保angle1到angle2是正向（逆時針）
            if angle2 < angle1:
                angle2 += 2 * math.pi
        else:
            # 需要確保angle1到angle2是負向（順時針）
            if angle1 < angle2:
                angle1 += 2 * math.pi
            # 交換，確保總是按照遠離線段的方向走
            angle1, angle2 = angle2, angle1

        # 擴展角度範圍：在交點之外額外延伸15度
        angle_extension = math.radians(15)
        angle1_extended = angle1 - angle_extension
        angle2_extended = angle2 + angle_extension

        # 計算擴展後的圓弧角度差
        angle_diff = abs(angle2_extended - angle1_extended)

        # 計算航點數量（每15度一個點）
        num_points = max(3, int(angle_diff / math.radians(15)))

        # 生成沿著圓周的規律航點（從擴展的起點到擴展的終點）
        detour_points = []
        for i in range(num_points + 1):
            t = i / num_points
            angle = angle1_extended + (angle2_extended - angle1_extended) * t

            # 在安全半徑上生成點
            x = safe_radius * math.cos(angle)
            y = safe_radius * math.sin(angle)

            point = to_latlon(x, y)
            detour_points.append(point)

        logger.info(f"規律繞行: 圓弧{math.degrees(angle_diff):.1f}度 (擴展+30度), "
                   f"{len(detour_points)}個航點, 半徑={safe_radius:.1f}m")

        return detour_points
    
    
    def _project_point_to_segment(self, point: Tuple[float, float], 
                                   p1: Tuple[float, float], 
                                   p2: Tuple[float, float]) -> Optional[Tuple[float, float]]:
        """
        將點投影到線段上，返回最近的點
        
        參數:
            point: 要投影的點
            p1, p2: 線段的兩個端點
        
        返回:
            線段上最接近point的點，如果線段退化為點則返回None
        """
        lat1, lon1 = p1
        lat2, lon2 = p2
        px, py = point
        
        # 向量計算
        dx = lon2 - lon1
        dy = lat2 - lat1
        
        # 線段長度平方
        length_sq = dx * dx + dy * dy
        
        if length_sq < 1e-10:  # 線段退化為點
            return None
        
        # 計算投影參數 t
        t = ((px - lon1) * dx + (py - lat1) * dy) / length_sq
        
        # 限制在線段範圍內
        t = max(0.0, min(1.0, t))
        
        # 計算投影點
        proj_lon = lon1 + t * dx
        proj_lat = lat1 + t * dy
        
        return (proj_lat, proj_lon)
    
    def _interpolate_point(self, p1: Tuple[float, float], 
                          p2: Tuple[float, float], 
                          t: float) -> Tuple[float, float]:
        """
        在兩點之間進行線性插值
        
        參數:
            p1, p2: 線段的兩個端點
            t: 插值參數，0表示p1，1表示p2
        
        返回:
            插值點
        """
        lat1, lon1 = p1
        lat2, lon2 = p2
        
        lat = lat1 + t * (lat2 - lat1)
        lon = lon1 + t * (lon2 - lon1)
        
        return (lat, lon)
    
    def point_in_polygon(self, point: Tuple[float, float], 
                        polygon: List[Tuple[float, float]]) -> bool:
        """射線法判斷點是否在多邊形內"""
        lat, lon = point
        n = len(polygon)
        inside = False
        
        p1_lat, p1_lon = polygon[0]
        for i in range(1, n + 1):
            p2_lat, p2_lon = polygon[i % n]
            if lon > min(p1_lon, p2_lon):
                if lon <= max(p1_lon, p2_lon):
                    if lat <= max(p1_lat, p2_lat):
                        if p1_lon != p2_lon:
                            xinters = (lon - p1_lon) * (p2_lat - p1_lat) / (p2_lon - p1_lon) + p1_lat
                        if p1_lat == p2_lat or lat <= xinters:
                            inside = not inside
            p1_lat, p1_lon = p2_lat, p2_lon
        
        return inside
    
    def check_waypoint_collision(self, waypoint: Tuple[float, float]) -> bool:
        """檢查航點是否與任何障礙物衝突"""
        for obstacle in self.obstacles:
            distance = self.calculate_distance(waypoint, obstacle.center)
            if distance < obstacle.effective_radius:
                return True
        return False
    
    def check_segment_collision(self, p1: Tuple[float, float], 
                               p2: Tuple[float, float]) -> List[Obstacle]:
        """
        檢查線段是否穿過障礙物
        返回: 與線段碰撞的障礙物列表
        """
        colliding_obstacles = []
        for obstacle in self.obstacles:
            if self.line_intersects_circle(p1, p2, obstacle.center, 
                                          obstacle.effective_radius):
                colliding_obstacles.append(obstacle)
        return colliding_obstacles
    
    def calculate_distance(self, p1: Tuple[float, float], 
                          p2: Tuple[float, float]) -> float:
        """計算兩點間距離（公尺）- 平面近似"""
        lat1, lon1 = p1
        lat2, lon2 = p2
        
        # 使用平均緯度計算經度縮放
        avg_lat = (lat1 + lat2) / 2
        cos_lat = math.cos(math.radians(avg_lat))
        
        # 緯度和經度差值（度）
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        
        # 轉換為公尺
        dy = dlat * self.earth_radius_m
        dx = dlon * self.earth_radius_m * cos_lat
        
        # 計算距離
        distance = math.sqrt(dx*dx + dy*dy)
        return distance
    
    def line_intersects_circle(self, p1: Tuple[float, float], 
                               p2: Tuple[float, float],
                               center: Tuple[float, float], 
                               radius_m: float) -> bool:
        """
        檢測線段是否與圓相交
        
        使用點到線段的最短距離算法
        """
        lat1, lon1 = p1
        lat2, lon2 = p2
        cx, cy = center
        
        # 使用與waypoint_generator相同的座標轉換
        cosLat0 = math.cos(math.radians(cx))
        
        def to_meters(lat, lon):
            # 與waypoint_generator.py的project_and_rotate一致
            x = (lon - cy) * self.earth_radius_m * cosLat0
            y = (lat - cx) * self.earth_radius_m
            return x, y

        x1, y1 = to_meters(lat1, lon1)
        x2, y2 = to_meters(lat2, lon2)
        
        dx = x2 - x1
        dy = y2 - y1
        dr2 = dx*dx + dy*dy
        
        if dr2 == 0:
            # 線段退化為點
            return (x1*x1 + y1*y1) <= radius_m*radius_m

        # 計算圓心到線段的最短距離
        t = -(x1*dx + y1*dy) / dr2
        t = max(0, min(1, t))  # 限制在線段上
        
        closest_x = x1 + t * dx
        closest_y = y1 + t * dy
        
        dist_sq = closest_x*closest_x + closest_y*closest_y
        
        return dist_sq < (radius_m * radius_m)
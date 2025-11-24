"""
地圖管理器模組
負責地圖相關操作，包括初始化、服務器切換等
"""

import threading

from config import Config
from logger_utils import logger


# ==============================
# 地圖管理器
# ==============================
class MapManager:
    """地圖管理器 - 負責地圖相關操作"""
    
    def __init__(self, map_widget):
        self.map = map_widget
        self.current_server = 0
        
    def initialize_map(self):
        """初始化地圖"""
        try:
            # 直接同步載入地圖（更可靠）
            success = False

            # 嘗試載入地圖伺服器
            for i, (name, url, max_zoom) in enumerate(Config.MAP_SERVERS):
                try:
                    logger.info(f"嘗試載入地圖伺服器: {name}")
                    self.map.set_tile_server(url, max_zoom=max_zoom)
                    self.current_server = i
                    logger.info(f"成功載入地圖伺服器: {name}")
                    success = True
                    break
                except Exception as e:
                    logger.warning(f"地圖伺服器 {name} 載入失敗: {e}")
                    continue

            if not success:
                logger.error("所有地圖伺服器載入失敗，使用預設瓦片")
                # 使用預設的OpenStreetMap
                self.map.set_tile_server("https://tile.openstreetmap.org/{z}/{x}/{y}.png", max_zoom=19)

            # 設定預設位置
            self.map.set_position(*Config.DEFAULT_POSITION)
            self.map.set_zoom(Config.DEFAULT_ZOOM)
            logger.info(f"地圖位置設定完成: {Config.DEFAULT_POSITION}, 縮放: {Config.DEFAULT_ZOOM}")

        except Exception as e:
            logger.error(f"地圖初始化失敗: {e}", exc_info=True)
    
    def switch_map_server(self, server_index: int):
        """切換地圖伺服器"""
        try:
            if 0 <= server_index < len(Config.MAP_SERVERS):
                name, url, max_zoom = Config.MAP_SERVERS[server_index]
                self.map.set_tile_server(url, max_zoom=max_zoom)
                self.current_server = server_index
                logger.info(f"切換到地圖伺服器: {name}")
        except Exception as e:
            logger.error(f"切換地圖伺服器失敗: {e}")
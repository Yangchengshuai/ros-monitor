from typing import Dict, Set, List, Any
from fastapi import WebSocket
import logging

logger = logging.getLogger(__name__)

class ConnectionManager:
    def __init__(self) -> None:
        self.clients: Dict[str, WebSocket] = {}
        self.subscriptions: Dict[str, Set[str]] = {}
        self.client_info: Dict[str, Dict[str, Any]] = {}

    async def connect(self, ws: WebSocket, client_id: str) -> None:
        """接受WebSocket连接"""
        try:
            await ws.accept()
            self.clients[client_id] = ws
            self.subscriptions.setdefault(client_id, set())
            self.client_info[client_id] = {
                "connected_at": None,
                "last_activity": None,
                "subscription_count": 0
            }
            
            # 发送连接确认消息
            await ws.send_json({
                "type": "connected", 
                "client_id": client_id,
                "message": "WebSocket连接已建立"
            })
            
            logger.info(f"Client {client_id} connected. Total clients: {len(self.clients)}")
            
        except Exception as e:
            logger.error(f"Failed to accept connection from {client_id}: {e}")
            raise

    def disconnect(self, client_id: str) -> None:
        """断开WebSocket连接"""
        try:
            self.clients.pop(client_id, None)
            self.subscriptions.pop(client_id, None)
            self.client_info.pop(client_id, None)
            logger.info(f"Client {client_id} disconnected. Total clients: {len(self.clients)}")
        except Exception as e:
            logger.error(f"Error during disconnect for {client_id}: {e}")

    async def subscribe_topics(self, client_id: str, topics: List[str]) -> None:
        """订阅话题"""
        try:
            subs = self.subscriptions.setdefault(client_id, set())
            subs.update(topics)
            
            # 更新客户端信息
            if client_id in self.client_info:
                self.client_info[client_id]["subscription_count"] = len(subs)
                self.client_info[client_id]["last_activity"] = "subscribe"
            
            ws = self.clients.get(client_id)
            if ws:
                await ws.send_json({
                    "type": "subscribed", 
                    "topics": list(subs),
                    "message": f"已订阅话题: {', '.join(topics)}"
                })
                logger.info(f"Client {client_id} subscribed to topics: {topics}")
        except Exception as e:
            logger.error(f"Error subscribing {client_id} to topics {topics}: {e}")

    async def unsubscribe_topics(self, client_id: str, topics: List[str]) -> None:
        """取消订阅话题"""
        try:
            subs = self.subscriptions.setdefault(client_id, set())
            for topic in topics:
                subs.discard(topic)
            
            # 更新客户端信息
            if client_id in self.client_info:
                self.client_info[client_id]["subscription_count"] = len(subs)
                self.client_info[client_id]["last_activity"] = "unsubscribe"
            
            ws = self.clients.get(client_id)
            if ws:
                await ws.send_json({
                    "type": "unsubscribed", 
                    "topics": list(subs),
                    "message": f"已取消订阅话题: {', '.join(topics)}"
                })
                logger.info(f"Client {client_id} unsubscribed from topics: {topics}")
        except Exception as e:
            logger.error(f"Error unsubscribing {client_id} from topics {topics}: {e}")

    async def send_personal_message(self, message: Dict[str, Any], client_id: str) -> bool:
        """发送个人消息给指定客户端"""
        try:
            ws = self.clients.get(client_id)
            if ws and ws.client_state.value == 1:  # 检查连接状态
                await ws.send_json(message)
                return True
            else:
                logger.warning(f"Client {client_id} not found or not connected")
                return False
        except Exception as e:
            logger.error(f"Error sending personal message to {client_id}: {e}")
            # 如果发送失败，断开连接
            await self._handle_client_error(client_id, e)
            return False

    async def broadcast_to_subscribers(self, topic: str, payload: Dict[str, Any]) -> int:
        """广播消息给订阅了指定话题的客户端"""
        success_count = 0
        failed_clients = []
        
        for client_id, ws in list(self.clients.items()):
            if topic in self.subscriptions.get(client_id, set()):
                try:
                    if ws.client_state.value == 1:  # 检查连接状态
                        # 直接发送payload，避免双重嵌套
                        await ws.send_json(payload)
                        success_count += 1
                    else:
                        failed_clients.append(client_id)
                except Exception as e:
                    logger.error(f"Error broadcasting to {client_id}: {e}")
                    failed_clients.append(client_id)
        
        # 清理失败的客户端
        for client_id in failed_clients:
            await self._handle_client_error(client_id, Exception("Broadcast failed"))
        
        if failed_clients:
            logger.warning(f"Failed to broadcast to clients: {failed_clients}")
        
        return success_count

    async def broadcast_to_all(self, message: Dict[str, Any]) -> int:
        """广播消息给所有连接的客户端"""
        success_count = 0
        failed_clients = []
        
        for client_id, ws in list(self.clients.items()):
            try:
                if ws.client_state.value == 1:  # 检查连接状态
                    await ws.send_json(message)
                    success_count += 1
                else:
                    failed_clients.append(client_id)
            except Exception as e:
                logger.error(f"Error broadcasting to {client_id}: {e}")
                failed_clients.append(client_id)
        
        # 清理失败的客户端
        for client_id in failed_clients:
            await self._handle_client_error(client_id, Exception("Broadcast failed"))
        
        return success_count

    async def _handle_client_error(self, client_id: str, error: Exception) -> None:
        """处理客户端错误"""
        try:
            logger.warning(f"Handling error for client {client_id}: {error}")
            self.disconnect(client_id)
        except Exception as e:
            logger.error(f"Error handling client error for {client_id}: {e}")

    def get_client_count(self) -> int:
        """获取当前连接的客户端数量"""
        return len(self.clients)

    def get_subscription_info(self) -> Dict[str, Any]:
        """获取订阅信息统计"""
        topic_subscribers = {}
        for client_id, topics in self.subscriptions.items():
            for topic in topics:
                if topic not in topic_subscribers:
                    topic_subscribers[topic] = []
                topic_subscribers[topic].append(client_id)
        
        return {
            "total_clients": len(self.clients),
            "topic_subscribers": topic_subscribers,
            "client_info": self.client_info
        }

    def is_client_connected(self, client_id: str) -> bool:
        """检查客户端是否连接"""
        return client_id in self.clients

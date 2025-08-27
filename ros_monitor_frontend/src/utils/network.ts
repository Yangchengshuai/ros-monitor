// 网络工具函数

/**
 * 获取当前页面的主机地址
 * 支持本地开发和生产环境
 */
export function getCurrentHost(): string {
  // 优先使用环境变量配置
  const envHost = import.meta.env.VITE_API_HOST;
  if (envHost && envHost !== 'localhost') {
    return envHost;
  }

  // 从当前页面URL获取主机地址
  const currentHost = window.location.hostname;
  
  // 如果是localhost或127.0.0.1，尝试获取局域网IP
  if (currentHost === 'localhost' || currentHost === '127.0.0.1') {
    // 检查是否有环境变量配置的局域网IP
    const lanIP = import.meta.env.VITE_LAN_IP;
    if (lanIP) {
      return lanIP;
    }
    
    // 默认返回localhost（开发环境）
    return 'localhost';
  }
  
  return currentHost;
}

/**
 * 获取WebSocket连接地址
 */
export function getWebSocketUrl(port?: number): string {
  const host = getCurrentHost();
  const wsPort = port || import.meta.env.VITE_WS_PORT || 8000;
  return `ws://${host}:${wsPort}`;
}

/**
 * 获取API基础URL
 */
export function getApiBaseUrl(port?: number): string {
  const host = getCurrentHost();
  const apiPort = port || import.meta.env.VITE_API_PORT || 8000;
  return `http://${host}:${apiPort}`;
}

/**
 * 检测是否为局域网访问
 */
export function isLocalNetworkAccess(): boolean {
  const host = getCurrentHost();
  return host !== 'localhost' && host !== '127.0.0.1';
}

/**
 * 获取连接状态信息
 */
export function getConnectionInfo() {
  const host = getCurrentHost();
  const isLocal = !isLocalNetworkAccess();
  
  return {
    host,
    isLocal,
    wsUrl: getWebSocketUrl(),
    apiUrl: getApiBaseUrl(),
    displayText: isLocal ? 
      `本地开发环境 (${host})` : 
      `局域网访问 (${host})`
  };
}

/**
 * 生成局域网访问提示信息
 */
export function getNetworkAccessTip(): string {
  const info = getConnectionInfo();
  
  if (info.isLocal) {
    return `当前为本地开发环境，局域网访问请修改环境变量 VITE_LAN_IP 为服务器IP地址`;
  }
  
  return `当前通过局域网访问，服务器地址: ${info.host}`;
}

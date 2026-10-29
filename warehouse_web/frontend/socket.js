export class StateConnection {
  constructor(onState, onConnection) {
    this.onState = onState;
    this.onConnection = onConnection;
    this.socket = null;
    this.stopped = false;
    this.attempt = 0;
  }

  start() {
    if (this.stopped || this.socket) return;
    this.onConnection('connecting');
    const protocol = location.protocol === 'https:' ? 'wss:' : 'ws:';
    const socket = this.socket = new WebSocket(`${protocol}//${location.host}/ws/state`);
    this.lastMessage = performance.now();
    this.watchdog = setInterval(() => {
      if (performance.now() - this.lastMessage > 5000) socket.close();
    }, 1000);
    socket.onmessage = event => {
      try {
        const value = JSON.parse(event.data);
        if (value.schema_version !== 1 || !Array.isArray(value.robots) ||
            !Number.isFinite(value.stale_after)) throw new Error('Unsupported state schema');
        this.lastMessage = performance.now();
        this.attempt = 0;
        this.onConnection('online');
        this.onState(value);
      } catch (error) {
        console.error('Invalid telemetry frame', error);
        socket.close();
      }
    };
    socket.onerror = () => socket.close();
    socket.onclose = () => {
      clearInterval(this.watchdog);
      this.socket = null;
      this.onConnection('offline');
      if (!this.stopped) {
        const delay = Math.min(10000, 500 * 2 ** this.attempt++) + Math.random() * 200;
        this.retry = setTimeout(() => this.start(), delay);
      }
    };
  }

  stop() {
    this.stopped = true;
    clearTimeout(this.retry);
    clearInterval(this.watchdog);
    this.socket?.close();
  }
}

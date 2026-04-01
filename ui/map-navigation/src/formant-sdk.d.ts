declare module "@formant/data-sdk" {
  export interface RealtimeStreamSource {
    id?: string;
    sourceType: "realtime";
    rosTopicName: string;
    streamType?: string;
  }

  export interface Device {
    id: string;
    name: string;
    getAvailableCommands(): Promise<Array<{ name: string }>>;
    sendCommand(name: string, data?: string): Promise<void>;
  }

  export interface IStreamData {
    deviceId: string;
    name: string;
    type: string;
    tags: Record<string, string | number>;
    points: Array<[number, unknown]>;
  }

  export class App {
    static getCurrentModuleConfiguration(): Promise<string | undefined>;
    static addModuleConfigurationListener(
      handler: (event: { configuration: string }) => void
    ): () => void;
    static showMessage(message: string): void;
  }

  export const Authentication: {
    waitTilAuthenticated(): Promise<boolean>;
  };

  export const Fleet: {
    getCurrentDevice(): Promise<Device>;
    queryTelemetry(query: {
      deviceIds: string[];
      names: string[];
      types: string[];
      start: string;
      end: string;
    }): Promise<IStreamData[]>;
  };

  export class LiveUniverseData {
    subscribeToJson(
      deviceId: string,
      source: RealtimeStreamSource,
      callback: (data: unknown) => void
    ): () => void;
    subscribeToVideo(
      deviceId: string,
      source: RealtimeStreamSource,
      callback: (canvas: HTMLCanvasElement) => void
    ): () => void;
  }
}

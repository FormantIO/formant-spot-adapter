const LOG_PREFIX = "[spot-nav]";

type FormantEnvironment = "prod" | "dev" | "stage";

interface FormantEnvironmentDetection {
  environment: FormantEnvironment;
  matchedSource: string;
  matchedHost: string;
}

interface FormantSdkModule {
  Authentication: {
    loginWithToken(token: string): Promise<void>;
  };
  DataSdk: {
    init(config: {
      adminApi: string;
      queryApi: string;
      ingestionApi: string;
      signalingApi: string;
    }): void;
  };
}

function unique(values: string[]): string[] {
  return Array.from(new Set(values.filter(Boolean)));
}

function stripAuthTokenFromUrl(): string | undefined {
  if (typeof window === "undefined") return undefined;
  const url = new URL(window.location.href);
  const authToken = url.searchParams.get("auth") ?? undefined;
  if (!authToken) return undefined;
  url.searchParams.delete("auth");
  window.history.replaceState(window.history.state, document.title, url.toString());
  return authToken;
}

function decodeJwtPayload(token: string): Record<string, unknown> | undefined {
  try {
    const [, payload] = token.split(".");
    if (!payload) return undefined;
    const normalized = payload.replace(/-/g, "+").replace(/_/g, "/");
    const padded = normalized.padEnd(Math.ceil(normalized.length / 4) * 4, "=");
    const json = window.atob(padded);
    return JSON.parse(json) as Record<string, unknown>;
  } catch {
    return undefined;
  }
}

function detectEnvironmentFromHost(hostname: string): FormantEnvironment | undefined {
  const host = hostname.toLowerCase();
  if (
    host.includes("iam-dev.formant.io") ||
    host.includes("api-dev.formant.io") ||
    host.includes("app-dev.formant.io") ||
    host.includes("platform-dev.formant.io")
  ) {
    return "dev";
  }
  if (
    host.includes("iam-stage.formant.io") ||
    host.includes("api-stage.formant.io") ||
    host.includes("app-stage.formant.io") ||
    host.includes("platform-stage.formant.io")
  ) {
    return "stage";
  }
  if (
    host.includes("api.formant.io") ||
    host.includes("app.formant.io") ||
    host.includes("platform.formant.io")
  ) {
    return "prod";
  }
  return undefined;
}

function collectDetectionCandidates(searchParams: URLSearchParams): string[] {
  const candidates: string[] = [];
  const explicit = searchParams.get("formant_env") ?? searchParams.get("env");
  if (explicit) {
    candidates.push(`env:${explicit}`);
  }

  if (typeof window !== "undefined") {
    candidates.push(window.location.href);
  }
  if (typeof document !== "undefined" && document.referrer) {
    candidates.push(document.referrer);
  }

  if (typeof location !== "undefined" && "ancestorOrigins" in location) {
    const ancestorOrigins = location.ancestorOrigins;
    for (let index = 0; index < ancestorOrigins.length; index += 1) {
      const origin = ancestorOrigins[index];
      if (origin) {
        candidates.push(origin);
      }
    }
  }

  const authToken = searchParams.get("auth");
  if (authToken) {
    const payload = decodeJwtPayload(authToken);
    if (payload && typeof payload.iss === "string") {
      candidates.push(payload.iss);
    }
  }

  return unique(candidates);
}

function detectFormantEnvironment(searchParams: URLSearchParams): FormantEnvironmentDetection {
  const explicit = searchParams.get("formant_env") ?? searchParams.get("env");
  if (explicit) {
    const normalized = explicit.trim().toLowerCase();
    if (normalized === "dev" || normalized === "stage" || normalized === "prod") {
      return {
        environment: normalized,
        matchedSource: `query:${explicit}`,
        matchedHost: normalized
      };
    }
  }

  const candidates = collectDetectionCandidates(searchParams);
  for (const candidate of candidates) {
    const rawHost = candidate.startsWith("env:")
      ? candidate.slice(4)
      : (() => {
          try {
            return new URL(candidate).hostname;
          } catch {
            return candidate;
          }
        })();
    const environment = detectEnvironmentFromHost(rawHost);
    if (environment) {
      return {
        environment,
        matchedSource: candidate,
        matchedHost: rawHost
      };
    }
  }

  return {
    environment: "prod",
    matchedSource: "default",
    matchedHost: "api.formant.io"
  };
}

function buildSdkConfig(environment: FormantEnvironment): {
  adminApi: string;
  queryApi: string;
  ingestionApi: string;
  signalingApi: string;
} {
  const signalingApi =
    environment === "prod"
      ? "https://api.formant.io"
      : `https://api-${environment}.formant.io`;

  return {
    adminApi: `${signalingApi}/v1/admin`,
    queryApi: `${signalingApi}/v1/queries`,
    ingestionApi: `${signalingApi}/v1/ingest`,
    signalingApi
  };
}

export async function initializeFormantSdkEnvironment(): Promise<void> {
  const authToken = stripAuthTokenFromUrl();
  const searchParams =
    typeof window !== "undefined"
      ? new URLSearchParams(window.location.search)
      : new URLSearchParams();
  const detection = detectFormantEnvironment(searchParams);
  const sdkConfig = buildSdkConfig(detection.environment);
  const sdk = (await import("@formant/data-sdk")) as FormantSdkModule;

  sdk.DataSdk.init(sdkConfig);

  console.info(`${LOG_PREFIX} sdk environment`, {
    environment: detection.environment,
    matchedSource: detection.matchedSource,
    matchedHost: detection.matchedHost,
    signalingApi: sdkConfig.signalingApi
  });

  if (!authToken) {
    console.info(`${LOG_PREFIX} sdk auth retry skipped`, {
      reason: "no auth token in url"
    });
    return;
  }

  try {
    await sdk.Authentication.loginWithToken(authToken);
    console.info(`${LOG_PREFIX} sdk auth retry succeeded`, {
      environment: detection.environment
    });
  } catch (error) {
    console.warn(`${LOG_PREFIX} sdk auth retry failed`, {
      environment: detection.environment,
      error: error instanceof Error ? error.message : String(error)
    });
  }
}

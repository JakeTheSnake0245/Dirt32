/**
 * Gateway proxy routes — expose Dirt32 gateway data (nodes + alerts,
 * including WiFi radar / CSI detections) through the typed API.
 *
 * The gateway daemon (Pi in the field, simulator in the Replit preview)
 * owns the SQLite DB; this server proxies and normalizes its `/gw/*` JSON
 * into the OpenAPI-defined camelCase shapes. Set GATEWAY_URL to point at
 * the daemon; defaults to the local preview gateway.
 */
import { Router, type IRouter } from "express";
import {
  ListGatewayNodesResponse,
  ListGatewayAlertsResponse,
  ListGatewayAlertsQueryParams,
} from "@workspace/api-zod";

const router: IRouter = Router();

const GATEWAY_URL = process.env.GATEWAY_URL ?? "http://127.0.0.1:80";

const EV_NAMES: Record<number, string> = {
  0: "unknown",
  1: "footstep",
  2: "vehicle",
  3: "multiple",
  4: "wifi_presence", // WiFi radar (CSI) presence detection
};

async function gwFetch(path: string): Promise<unknown> {
  const res = await fetch(`${GATEWAY_URL}${path}`);
  if (!res.ok) throw new Error(`gateway returned ${res.status}`);
  return res.json();
}

router.get("/gateway/nodes", async (_req, res) => {
  try {
    const raw = (await gwFetch("/gw/nodes")) as { nodes: Record<string, unknown>[] };
    const nodes = (raw.nodes ?? []).map((n) => ({
      nodeId: n.node_id as number,
      name: (n.name as string | null) ?? null,
      color: n.color as string,
      reasons: (n.reasons as string[]) ?? [],
      lat: (n.lat as number | null) ?? null,
      lon: (n.lon as number | null) ?? null,
      lastSeen: (n.last_seen as number | null) ?? null,
      batteryMv: (n.battery_mv as number | null) ?? null,
      noiseFloor: (n.noise_floor as number | null) ?? null,
      csiOn: Boolean(n.csi_on),
      csiCalibrating: Boolean(n.csi_calibrating),
      // Wire carries the metric x100; expose the real value.
      csiNoise: n.csi_noise == null ? null : (n.csi_noise as number) / 100,
    }));
    res.json(ListGatewayNodesResponse.parse({ nodes }));
  } catch (err) {
    res.status(502).json({ error: `gateway unreachable: ${String(err)}` });
  }
});

router.get("/gateway/alerts", async (req, res) => {
  const query = ListGatewayAlertsQueryParams.safeParse(req.query);
  if (!query.success) {
    res.status(400).json({ error: query.error.message });
    return;
  }
  const limit = query.data.limit ?? 100;
  try {
    const raw = (await gwFetch(`/gw/alerts?limit=${limit}`)) as {
      alerts: Record<string, unknown>[];
    };
    let alerts = (raw.alerts ?? []).map((a) => {
      const eventClass = EV_NAMES[a.event_class as number] ?? "unknown";
      return {
        nodeId: a.node_id as number,
        receivedAt: a.received_at as number,
        eventClass,
        channel: eventClass === "wifi_presence" ? "rf" : "seismic",
        confidence: a.confidence as number,
        peakAmp: a.peak_amp as number,
        batteryMv: (a.battery_mv as number | null) ?? null,
      };
    });
    if (query.data.eventClass)
      alerts = alerts.filter((a) => a.eventClass === query.data.eventClass);
    res.json(ListGatewayAlertsResponse.parse({ alerts }));
  } catch (err) {
    res.status(502).json({ error: `gateway unreachable: ${String(err)}` });
  }
});

export default router;

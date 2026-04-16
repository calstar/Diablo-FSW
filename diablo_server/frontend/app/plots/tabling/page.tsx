'use client'

import { useCallback, useEffect, useMemo, useState } from 'react';
import TimeSeriesPlot from '@/components/plots/TimeSeriesPlot';
import { useActuatorsFromConfig } from '@/lib/actuators-from-config';
import { useControlMode } from '@/lib/control-mode';
import { getEntityColor } from '@/lib/sensor-colors';
import {
  buildLcDataFromBoards,
  buildPtCalDataFromBoards,
  buildTcDataFromBoards,
  type RtdLcRowConfig,
  type TcRowConfig,
} from '@/lib/sensor-info-entities';
import { useSensorStore, useActuatorCommandedState } from '@/lib/store';
import { getApiBaseUrl, getWebSocketClient } from '@/lib/websocket';
import { ActuatorState, CommandPayload, MessageType } from '@/lib/types';

const TWELVE_VOLT_ACT_BOARD_ID = 12;
const PT_OPEN_THRESHOLD_PSI = 75;
const OPEN_COMMAND_INTERVAL_MS = 100;
const PLOT_WINDOW_SECONDS = 60;

function TablingActuatorSlotCard({
  name,
  channel,
  boardId,
  canControl,
}: {
  name: string;
  channel: number;
  boardId: number;
  canControl: boolean;
}) {
  const ws = getWebSocketClient();
  const boardNumber = (boardId % 10) || 10;
  const commandedEntity = `ACT_CMD.B${boardNumber}.CH${channel}`;
  const commanded = useActuatorCommandedState(commandedEntity);
  const [pending, setPending] = useState(false);

  const sendCommand = (state: ActuatorState) => {
    if (!canControl) return;
    const command: CommandPayload = {
      commandType: 'actuator',
      data: { actuatorName: name, actuatorState: state },
    };
    ws.sendCommand(command);
    setPending(true);
    window.setTimeout(() => setPending(false), 450);
  };

  const commandedOpen = commanded === ActuatorState.OPEN;
  const commandedClosed = commanded === ActuatorState.CLOSED;

  return (
    <div className="rounded border border-gray-700 hover:border-gray-600 transition-colors h-full min-h-[120px] flex flex-col relative p-0.5 bg-background max-w-xs">
      <div className="absolute top-0.5 right-0.5 flex items-center gap-0.5">
        {pending && <span className="text-yellow-400 text-[8px] leading-none">⟳</span>}
        <div
          className={`w-2 h-2 rounded-full flex-shrink-0 ${
            commanded === null ? 'bg-gray-600' : commandedOpen ? 'bg-green-500' : 'bg-red-500'
          }`}
        />
      </div>
      <div className="flex-1 flex items-center min-h-0 overflow-hidden pr-4 flex-shrink-0">
        <h3 className="font-bold tracking-wider text-text uppercase leading-tight truncate text-[9px] xl:text-[10px]">
          {name}
        </h3>
      </div>
      <div className="grid grid-cols-2 gap-0.5 flex-shrink-0 min-h-0">
        <button
          type="button"
          onClick={() => sendCommand(ActuatorState.OPEN)}
          disabled={!canControl}
          className={`h-full min-h-0 rounded text-[8px] xl:text-[9px] font-bold uppercase tracking-wider leading-none transition-all py-0.5
            ${
              commandedOpen
                ? canControl
                  ? 'bg-green-700 text-white ring-1 ring-green-400'
                  : 'bg-green-700/50 text-green-300 ring-1 ring-green-700 cursor-not-allowed'
                : canControl
                  ? 'bg-gray-800 hover:bg-gray-700 text-gray-300'
                  : 'bg-gray-900 text-gray-600 cursor-not-allowed opacity-50'
            }`}
        >
          Open
        </button>
        <button
          type="button"
          onClick={() => sendCommand(ActuatorState.CLOSED)}
          disabled={!canControl}
          className={`h-full min-h-0 rounded text-[8px] xl:text-[9px] font-bold uppercase tracking-wider leading-none transition-all py-0.5
            ${
              commandedClosed
                ? canControl
                  ? 'bg-red-700 text-white ring-1 ring-red-400'
                  : 'bg-red-700/50 text-red-300 ring-1 ring-red-700 cursor-not-allowed'
                : canControl
                  ? 'bg-gray-800 hover:bg-gray-700 text-gray-300'
                  : 'bg-gray-900 text-gray-600 cursor-not-allowed opacity-50'
            }`}
        >
          Close
        </button>
      </div>
    </div>
  );
}

function SensorPlotBlock({
  title,
  calEntity,
  label,
  component,
  yLabel,
  color,
}: {
  title: string;
  calEntity: string;
  label: string;
  component: string;
  yLabel: string;
  color: string;
}) {
  return (
    <div className="bg-card rounded-xl border border-gray-800 p-2 flex flex-col min-h-0 flex-1 min-h-[200px]">
      <h2 className="text-[10px] font-bold tracking-widest text-text-muted uppercase mb-2 leading-none flex-shrink-0">
        {title}
      </h2>
      <div className="flex-1 min-h-0 min-w-0">
        <TimeSeriesPlot
          title=""
          entities={[calEntity]}
          labels={[label]}
          component={component}
          colors={[color]}
          yLabel={yLabel}
          windowSeconds={PLOT_WINDOW_SECONDS}
        />
      </div>
    </div>
  );
}

export default function TablingPage() {
  const ws = getWebSocketClient();
  const debugMode = useSensorStore((s) => s.debugMode);
  const { controlEnabled } = useControlMode();
  const { actuators, loading: actuatorsLoading } = useActuatorsFromConfig();

  const actuators12 = useMemo(() => {
    return actuators
      .filter((a) => a.boardId === TWELVE_VOLT_ACT_BOARD_ID)
      .sort((a, b) => a.channel - b.channel);
  }, [actuators]);

  const [selectedChannel, setSelectedChannel] = useState<number | null>(null);
  useEffect(() => {
    if (selectedChannel == null && actuators12.length > 0) {
      setSelectedChannel(actuators12[0].channel);
    }
  }, [actuators12, selectedChannel]);

  const selectedActuator = useMemo(() => {
    if (selectedChannel == null) return null;
    return actuators12.find((a) => a.channel === selectedChannel) ?? null;
  }, [actuators12, selectedChannel]);

  const [tcRows, setTcRows] = useState<TcRowConfig[]>([]);
  const [ptRows, setPtRows] = useState<RtdLcRowConfig[]>([]);
  const [lcRows, setLcRows] = useState<RtdLcRowConfig[]>([]);
  const [tcIdx, setTcIdx] = useState(0);
  const [ptIdx, setPtIdx] = useState(0);
  const [lcIdx, setLcIdx] = useState(0);

  const loadBoards = useCallback(() => {
    fetch(`${getApiBaseUrl()}/api/config`)
      .then((r) => (r.ok ? r.json() : null))
      .then((data) => {
        const config = data?.config;
        const boards = config?.boards as Record<string, unknown> | undefined;
        const adc = config?.adc;
        if (adc && typeof adc.internal_v === 'number' && typeof adc.absolute_5v_v === 'number') {
          useSensorStore.getState().setVoltageRefNominals({ internalV: adc.internal_v, absolute5vV: adc.absolute_5v_v });
        }
        if (!boards) return;
        const tc = buildTcDataFromBoards(boards);
        const pt = buildPtCalDataFromBoards(boards);
        const lc = buildLcDataFromBoards(boards);
        if (tc.length) setTcRows(tc);
        else setTcRows([]);
        if (pt.length) setPtRows(pt);
        else setPtRows([]);
        if (lc.length) setLcRows(lc);
        else setLcRows([]);
      })
      .catch(() => {});
  }, []);

  useEffect(() => {
    loadBoards();
  }, [loadBoards]);

  useEffect(() => {
    const unsub = ws.on(MessageType.CONFIG_UPDATED, () => loadBoards());
    return () => {
      unsub();
    };
  }, [ws, loadBoards]);

  useEffect(() => {
    setTcIdx((i) => Math.min(i, Math.max(0, tcRows.length - 1)));
  }, [tcRows.length]);
  useEffect(() => {
    setPtIdx((i) => Math.min(i, Math.max(0, ptRows.length - 1)));
  }, [ptRows.length]);
  useEffect(() => {
    setLcIdx((i) => Math.min(i, Math.max(0, lcRows.length - 1)));
  }, [lcRows.length]);

  const tcSel = tcRows[tcIdx];
  const ptSel = ptRows[ptIdx];
  const lcSel = lcRows[lcIdx];

  const tcLabel = tcSel
    ? `TC Ch${tcSel.entity.match(/\.CH(\d+)/)?.[1] ?? '?'} (B${tcSel.boardId})`
    : '';
  const lcLabelFor = (row: RtdLcRowConfig) =>
    row.boardId != null
      ? `LC Ch${row.entity.match(/\.CH(\d+)/)?.[1] ?? '?'} (B${row.boardId})`
      : row.label;

  const canControl = debugMode && controlEnabled;

  const selectedPtCal = ptSel?.calEntity ?? '';

  useEffect(() => {
    if (!canControl || !selectedActuator?.name || !selectedPtCal) return;
    const id = window.setInterval(() => {
      const p = useSensorStore.getState().getSensorValue(selectedPtCal, 'pressure_psi');
      if (p == null || !Number.isFinite(p) || p <= PT_OPEN_THRESHOLD_PSI) return;
      const cmd: CommandPayload = {
        commandType: 'actuator',
        data: { actuatorName: selectedActuator.name, actuatorState: ActuatorState.OPEN },
      };
      getWebSocketClient().sendCommand(cmd);
    }, OPEN_COMMAND_INTERVAL_MS);
    return () => window.clearInterval(id);
  }, [canControl, selectedActuator?.name, selectedPtCal]);

  return (
    <main className="h-full min-h-0 bg-background text-text flex flex-col overflow-hidden p-3 gap-3">
      <div className="flex-shrink-0">
        <h1 className="text-lg font-bold tracking-wider text-text uppercase">Tabling</h1>
        <p className="text-xs text-text-muted mt-0.5">
          One-off demo: numeric 12V actuator slot, TC/PT/LC traces, and PT &gt; {PT_OPEN_THRESHOLD_PSI} PSI → OPEN
          (throttled {OPEN_COMMAND_INTERVAL_MS} ms).
        </p>
      </div>

      <div className="flex flex-col lg:flex-row gap-3 flex-shrink-0 items-start">
        <div className="bg-card rounded-xl border border-gray-800 p-2 flex flex-col gap-2 min-w-0 lg:max-w-md w-full">
          <h2 className="text-[10px] font-bold tracking-widest text-text-muted uppercase leading-none">
            12V actuator (board {TWELVE_VOLT_ACT_BOARD_ID})
          </h2>
          {actuatorsLoading ? (
            <p className="text-xs text-text-muted">Loading actuators…</p>
          ) : actuators12.length === 0 ? (
            <p className="text-xs text-amber-600/90">
              No actuator roles on board {TWELVE_VOLT_ACT_BOARD_ID} in config.toml actuator_roles.
            </p>
          ) : (
            <>
              <label className="text-[10px] text-text-muted uppercase tracking-wider">Slot (channel)</label>
              <select
                className="bg-gray-900 border border-gray-700 rounded px-2 py-1.5 text-sm text-text w-full max-w-xs"
                value={selectedChannel ?? ''}
                onChange={(e) => setSelectedChannel(Number(e.target.value))}
              >
                {actuators12.map((a) => (
                  <option key={a.channel} value={a.channel}>
                    Ch {a.channel}
                  </option>
                ))}
              </select>
              {!debugMode && (
                <p className="text-[10px] text-amber-600/90">Enable DEBUG in the top bar to command actuators.</p>
              )}
              {selectedActuator && (
                <TablingActuatorSlotCard
                  name={selectedActuator.name}
                  channel={selectedActuator.channel}
                  boardId={selectedActuator.boardId ?? TWELVE_VOLT_ACT_BOARD_ID}
                  canControl={canControl}
                />
              )}
            </>
          )}
        </div>
      </div>

      <div className="flex-1 min-h-0 grid grid-cols-1 lg:grid-cols-3 gap-3 overflow-auto">
        <div className="flex flex-col gap-2 min-h-0">
          <label className="text-[10px] text-text-muted uppercase tracking-wider px-1">Thermocouple</label>
          <select
            className="bg-gray-900 border border-gray-700 rounded px-2 py-1.5 text-sm text-text w-full"
            value={tcRows.length ? tcIdx : 0}
            onChange={(e) => setTcIdx(Number(e.target.value))}
            disabled={tcRows.length === 0}
          >
            {tcRows.map((row, i) => (
              <option key={row.calEntity} value={i}>
                {`TC Ch${row.entity.match(/\.CH(\d+)/)?.[1] ?? '?'} (B${row.boardId})`}
              </option>
            ))}
          </select>
          {tcSel ? (
            <SensorPlotBlock
              title="TC temperature"
              calEntity={tcSel.calEntity}
              label={tcLabel}
              component="temperature_c"
              yLabel="Temperature (°C)"
              color={getEntityColor(tcSel.calEntity)}
            />
          ) : (
            <p className="text-xs text-text-muted px-1">No TC boards in config.</p>
          )}
        </div>

        <div className="flex flex-col gap-2 min-h-0">
          <label className="text-[10px] text-text-muted uppercase tracking-wider px-1">Pressure (PT)</label>
          <select
            className="bg-gray-900 border border-gray-700 rounded px-2 py-1.5 text-sm text-text w-full"
            value={ptRows.length ? ptIdx : 0}
            onChange={(e) => setPtIdx(Number(e.target.value))}
            disabled={ptRows.length === 0}
          >
            {ptRows.map((row, i) => (
              <option key={row.calEntity} value={i}>
                {row.label}
              </option>
            ))}
          </select>
          {ptSel ? (
            <SensorPlotBlock
              title="PT pressure"
              calEntity={ptSel.calEntity}
              label={ptSel.label}
              component="pressure_psi"
              yLabel="Pressure (PSI)"
              color={getEntityColor(ptSel.calEntity)}
            />
          ) : (
            <p className="text-xs text-text-muted px-1">No PT boards in config.</p>
          )}
        </div>

        <div className="flex flex-col gap-2 min-h-0">
          <label className="text-[10px] text-text-muted uppercase tracking-wider px-1">Load cell</label>
          <select
            className="bg-gray-900 border border-gray-700 rounded px-2 py-1.5 text-sm text-text w-full"
            value={lcRows.length ? lcIdx : 0}
            onChange={(e) => setLcIdx(Number(e.target.value))}
            disabled={lcRows.length === 0}
          >
            {lcRows.map((row, i) => (
              <option key={row.calEntity} value={i}>
                {lcLabelFor(row)}
              </option>
            ))}
          </select>
          {lcSel ? (
            <SensorPlotBlock
              title="LC force"
              calEntity={lcSel.calEntity}
              label={lcLabelFor(lcSel)}
              component="force_kg"
              yLabel="Force (kg)"
              color={getEntityColor(lcSel.calEntity)}
            />
          ) : (
            <p className="text-xs text-text-muted px-1">No LC boards in config.</p>
          )}
        </div>
      </div>
    </main>
  );
}

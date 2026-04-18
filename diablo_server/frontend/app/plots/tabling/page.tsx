'use client'

import { useCallback, useEffect, useMemo, useState } from 'react';
import TimeSeriesPlot from '@/components/plots/TimeSeriesPlot';
import { useActuatorsFromConfig, type ActuatorFromConfig } from '@/lib/actuators-from-config';
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

function TablingActuatorRow({
  actuators,
  selectedChannel,
  onChannelChange,
  canControl,
  boardIdFallback,
}: {
  actuators: ActuatorFromConfig[];
  selectedChannel: number | null;
  onChannelChange: (channel: number) => void;
  canControl: boolean;
  boardIdFallback: number;
}) {
  const ws = getWebSocketClient();
  const selected = selectedChannel != null ? actuators.find((a) => a.channel === selectedChannel) : null;
  const boardId = selected?.boardId ?? boardIdFallback;
  const boardNumber = (boardId % 10) || 10;
  const channel = selected?.channel ?? 0;
  const name = selected?.name ?? '';

  const commandedEntity = selected ? `ACT_CMD.B${boardNumber}.CH${channel}` : '';
  const commanded = useActuatorCommandedState(commandedEntity);
  const [pending, setPending] = useState(false);

  const sendCommand = (state: ActuatorState) => {
    if (!canControl || !name) return;
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

  const openBase =
    'min-h-[3.5rem] px-4 sm:px-6 rounded-xl border font-bold uppercase tracking-widest text-base sm:text-lg transition-[transform,box-shadow] duration-150 select-none cursor-pointer disabled:cursor-not-allowed';
  const openActive =
    commandedOpen && canControl
      ? 'bg-green-700 text-white border-green-500 ring-2 ring-green-400/90 shadow-[0_0_12px_rgba(34,197,94,0.35)]'
      : commandedOpen && !canControl
        ? 'bg-green-900/50 text-green-300/80 border-green-800 ring-1 ring-green-900 cursor-not-allowed'
        : canControl
          ? 'bg-gray-800/90 text-gray-200 border-gray-600 hover:bg-green-900/40 hover:border-green-700 hover:text-green-100'
          : 'bg-gray-900 text-gray-600 border-gray-800 cursor-not-allowed opacity-60';
  const openPress =
    canControl && commandedOpen
      ? 'active:scale-[0.98] active:ring-2 active:ring-green-300 active:ring-offset-2 active:ring-offset-background'
      : canControl
        ? 'active:scale-[0.98] active:ring-2 active:ring-green-500/70 active:ring-offset-2 active:ring-offset-background'
        : '';

  const closeBase =
    'min-h-[3.5rem] px-4 sm:px-6 rounded-xl border font-bold uppercase tracking-widest text-base sm:text-lg transition-[transform,box-shadow] duration-150 select-none cursor-pointer disabled:cursor-not-allowed';
  const closeActive =
    commandedClosed && canControl
      ? 'bg-red-700 text-white border-red-500 ring-2 ring-red-400/90 shadow-[0_0_12px_rgba(239,68,68,0.35)]'
      : commandedClosed && !canControl
        ? 'bg-red-900/50 text-red-300/80 border-red-800 ring-1 ring-red-900 cursor-not-allowed'
        : canControl
          ? 'bg-gray-800/90 text-gray-200 border-gray-600 hover:bg-red-900/40 hover:border-red-700 hover:text-red-100'
          : 'bg-gray-900 text-gray-600 border-gray-800 cursor-not-allowed opacity-60';
  const closePress =
    canControl && commandedClosed
      ? 'active:scale-[0.98] active:ring-2 active:ring-red-300 active:ring-offset-2 active:ring-offset-background'
      : canControl
        ? 'active:scale-[0.98] active:ring-2 active:ring-red-500/70 active:ring-offset-2 active:ring-offset-background'
        : '';

  if (actuators.length === 0) return null;

  return (
    <div className="flex w-full min-w-0 flex-row flex-nowrap items-stretch gap-2 overflow-x-auto pb-0.5 sm:gap-3 [scrollbar-width:thin]">
      <div className="flex min-w-[12rem] max-w-[min(100%,22rem)] flex-[1_0_auto] items-center gap-2 sm:min-w-[14rem]">
        <select
          aria-label="Actuator slot"
          className="min-h-[3.5rem] min-w-0 flex-1 rounded-lg border border-gray-700 bg-gray-900 px-3 py-2 text-sm font-medium text-text"
          value={selectedChannel ?? ''}
          onChange={(e) => onChannelChange(Number(e.target.value))}
        >
          {actuators.map((a) => (
            <option key={a.channel} value={a.channel}>
              Ch {a.channel} — {a.name}
            </option>
          ))}
        </select>
        <div className="flex items-center gap-1.5 flex-shrink-0 px-0.5" title="Commanded state">
          {pending && <span className="text-yellow-400 text-sm leading-none">⟳</span>}
          <div
            className={`h-2.5 w-2.5 flex-shrink-0 rounded-full ring-1 ring-black/20 ${
              commanded === null ? 'bg-gray-600' : commandedOpen ? 'bg-green-500 shadow-[0_0_6px_rgba(34,197,94,0.8)]' : 'bg-red-500 shadow-[0_0_6px_rgba(239,68,68,0.8)]'
            }`}
          />
        </div>
      </div>
      <button
        type="button"
        onClick={() => sendCommand(ActuatorState.OPEN)}
        disabled={!canControl || !selected}
        className={`${openBase} min-w-[8.5rem] flex-[1_0_auto] sm:min-w-[10rem] ${openActive} ${openPress}`}
      >
        Open
      </button>
      <button
        type="button"
        onClick={() => sendCommand(ActuatorState.CLOSED)}
        disabled={!canControl || !selected}
        className={`${closeBase} min-w-[8.5rem] flex-[1_0_auto] sm:min-w-[10rem] ${closeActive} ${closePress}`}
      >
        Close
      </button>
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

  /** When true (default for this demo), PT>75 PSI sends OPEN on an interval. Turn off to test Open/Close without fighting pressure. */
  const [ptInterlockEnabled, setPtInterlockEnabled] = useState(true);

  useEffect(() => {
    if (!ptInterlockEnabled || !canControl || !selectedActuator?.name || !selectedPtCal) return;
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
  }, [ptInterlockEnabled, canControl, selectedActuator?.name, selectedPtCal]);

  return (
    <main className="h-full min-h-0 bg-background text-text flex flex-col overflow-hidden p-3 gap-3">
      <div className="flex-shrink-0">
        <h1 className="text-lg font-bold tracking-wider text-text uppercase">Tabling</h1>
        <p className="text-xs text-text-muted mt-0.5">
          One-off demo: numeric 12V actuator slot, TC/PT/LC traces, and (while this page is open) PT &gt;{' '}
          {PT_OPEN_THRESHOLD_PSI} PSI → OPEN every {OPEN_COMMAND_INTERVAL_MS} ms. Uncheck below to tune Open/Close without
          that loop — it will fight manual Close while pressure stays high.
        </p>
        <label className="mt-2 flex cursor-pointer items-center gap-2 text-xs text-text">
          <input
            type="checkbox"
            className="h-4 w-4 rounded border-gray-600 bg-gray-900"
            checked={ptInterlockEnabled}
            onChange={(e) => setPtInterlockEnabled(e.target.checked)}
          />
          <span>
            Enable PT &gt; {PT_OPEN_THRESHOLD_PSI} PSI auto-open (sends OPEN every {OPEN_COMMAND_INTERVAL_MS} ms while
            above threshold)
          </span>
        </label>
      </div>

      <div className="flex flex-col lg:flex-row gap-3 flex-shrink-0 items-start">
        <div className="bg-card rounded-xl border border-gray-800 p-3 flex flex-col gap-3 min-w-0 w-full">
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
              {!debugMode && (
                <p className="text-[10px] text-amber-600/90">Enable DEBUG in the top bar to command actuators.</p>
              )}
              <TablingActuatorRow
                actuators={actuators12}
                selectedChannel={selectedChannel}
                onChannelChange={setSelectedChannel}
                canControl={canControl}
                boardIdFallback={TWELVE_VOLT_ACT_BOARD_ID}
              />
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

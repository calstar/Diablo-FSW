'use client'

import { useCallback, useEffect, useState } from 'react';
import TimeSeriesPlot from '@/components/plots/TimeSeriesPlot';
import DerivedTimeSeriesPlot from '@/components/plots/DerivedTimeSeriesPlot';
import ActuatorStatePanel from '@/components/plots/ActuatorStatePanel';
import { MovingAverageReadout, useMovingAverage, useMovingAverageDerived } from '@/components/plots/MovingAverageReadout';
import { useSensorStore } from '@/lib/store';
import { getWebSocketClient } from '@/lib/websocket';
import { MessageType, SensorUpdate, StateUpdate } from '@/lib/types';
import { getEntityColor, getActuatorColor } from '@/lib/sensor-colors';
import { useSensorConfig, filterByRole } from '@/lib/sensor-config';
import { kTypeVoltageToTempC, codeToForce } from '@/lib/sense-conversions';

const ADC_FULL_SCALE = 2 ** 31;
const TC_REF_VOLTAGE = 2.5;
const SENSE_COLORS = ['#F59E0B', '#10B981', '#3B82F6', '#EC4899'];

function adcToVoltage(rawAdc: number, refVolts: number): number {
  const u = rawAdc >>> 0;
  const signed = u > 0x7fffffff ? u - 0x100000000 : u;
  return (signed / ADC_FULL_SCALE) * refVolts;
}

const LC_DEFAULTS = {
  adcRefVoltage: 3.3,
  excitationVoltage: 5,
  sensitivityMvPerV: 2,
  pgaGain: 128,
  fullScaleForceN: 1000,
};

function buildChannels(boards: Record<string, any>, type: 'TC' | 'LC'): number[] {
  const channels: number[] = [];
  for (const board of Object.values(boards)) {
    if (board.type !== type || board.enabled === false) continue;
    const active: number[] =
      Array.isArray(board.active_connectors) && board.active_connectors.length > 0
        ? (board.active_connectors as number[])
        : Array.from({ length: (board.num_sensors as number) ?? 10 }, (_, i) => i + 1);
    channels.push(...active);
  }
  return channels;
}

export default function ChamberGraphsPage() {
    const updateSensor = useSensorStore((s) => s.updateSensor);
    const updateState = useSensorStore((s) => s.updateState);
    const ws = getWebSocketClient();
    const allSensors = useSensorConfig();

    const ptSensors = filterByRole(allSensors, 'Upstream', 'Downstream');
    const fuelPtSensors = ptSensors.filter(s => s.role.toLowerCase().includes('fuel'));
    const loxPtSensors = ptSensors.filter(s => s.role.toLowerCase().includes('ox') || s.role.toLowerCase().includes('lox'));

    const fuelPtEntities = fuelPtSensors.map(s => s.calEntity);
    const loxPtEntities = loxPtSensors.map(s => s.calEntity);
    const ptEntities = ptSensors.map(s => s.calEntity);
    const ptLabels = ptSensors.map(s => s.role);
    const ptColors = ptEntities.map(e => getEntityColor(e));

    const [tcEntities, setTcEntities] = useState<string[]>([]);
    const [tcLabels, setTcLabels] = useState<string[]>([]);
    const [lcEntities, setLcEntities] = useState<string[]>([]);
    const [lcLabels, setLcLabels] = useState<string[]>([]);

    const loadTcLcConfig = useCallback(() => {
      fetch('/api/config')
        .then((r) => (r.ok ? r.json() : null))
        .then((data: any) => {
          const boards = data?.config?.boards ?? {};
          const tcCh = buildChannels(boards, 'TC');
          if (tcCh.length) {
            setTcEntities(tcCh.map((ch) => `TC.CH${ch}`));
            setTcLabels(tcCh.map((ch) => `TC Ch${ch}`));
          } else {
            setTcEntities([]);
            setTcLabels([]);
          }
          const lcCh = buildChannels(boards, 'LC');
          if (lcCh.length) {
            setLcEntities(lcCh.map((ch) => `LC.CH${ch}`));
            setLcLabels(lcCh.map((ch) => `LC Ch${ch}`));
          } else {
            setLcEntities([]);
            setLcLabels([]);
          }
        })
        .catch(() => {});
    }, []);

    useEffect(() => {
      loadTcLcConfig();
    }, [loadTcLcConfig]);

    const tcComponent = 'raw_adc_counts';
    const lcComponent = 'raw_adc_counts';
    const tcColors = SENSE_COLORS.slice(0, tcEntities.length);
    const lcColors = SENSE_COLORS.slice(0, lcEntities.length);

    const tcTransform = useCallback(
      (v: number) => kTypeVoltageToTempC(adcToVoltage(v, TC_REF_VOLTAGE)),
      []
    );
    const lcTransform = useCallback(
      (v: number) =>
        codeToForce(
          v,
          LC_DEFAULTS.adcRefVoltage,
          LC_DEFAULTS.excitationVoltage,
          LC_DEFAULTS.sensitivityMvPerV,
          LC_DEFAULTS.pgaGain,
          LC_DEFAULTS.fullScaleForceN
        ),
      []
    );

    const fuelPtMa = useMovingAverage(fuelPtEntities, 'pressure_psi', 5);
    const loxPtMa = useMovingAverage(loxPtEntities, 'pressure_psi', 5);
    const tcMa = useMovingAverageDerived(tcEntities, tcComponent, tcTransform, 5);
    const lcMa = useMovingAverageDerived(lcEntities, lcComponent, lcTransform, 5);

    useEffect(() => {
        ws.connect();
        const unsub1 = ws.on(MessageType.SENSOR_UPDATE, (p: unknown) => updateSensor(p as SensorUpdate));
        const unsub2 = ws.on(MessageType.STATE_UPDATE, (p: unknown) => updateState(p as StateUpdate));
        const unsub3 = ws.on(MessageType.CONFIG_UPDATED, () => loadTcLcConfig());
        return () => { unsub1(); unsub2(); unsub3(); };
    }, [ws, updateSensor, updateState, loadTcLcConfig]);

    return (
        <main className="h-full bg-background text-text flex flex-col overflow-hidden p-2 gap-1">

            <div className="flex items-center flex-shrink-0 justify-between gap-2">
                <div className="flex items-center gap-2">
                    <div className="w-0.5 h-4 bg-orange-500 rounded-full" />
                    <h1 className="text-sm font-bold text-orange-400 tracking-wider">CHAMBER SYSTEM</h1>
                </div>
                <div className="flex gap-1 bg-gray-900 rounded-lg p-0.5">
                    <div className="px-2 py-1 text-xs font-bold rounded bg-gray-800 text-gray-300">
                        Unified View
                    </div>
                </div>
            </div>

            <div className="flex-1 min-h-0 flex flex-col gap-1.5">
                {/* 3 columns: each with moving average at top, then graph */}
                <div className="flex-[5] min-h-0 flex flex-row gap-2 min-w-0">
                    <div className="flex-1 flex flex-col gap-1.5 min-h-0 min-w-0">
                        <div className="flex gap-2 flex-wrap">
                            <MovingAverageReadout
                                label="Fuel PT avg"
                                value={fuelPtMa}
                                unit="PSI"
                                decimals={1}
                            />
                            <MovingAverageReadout
                                label="LOX PT avg"
                                value={loxPtMa}
                                unit="PSI"
                                decimals={1}
                            />
                        </div>
                        <div className="flex-1 bg-card rounded-lg p-2 flex flex-col min-h-0 min-w-0">
                            <TimeSeriesPlot title="PT Pressures" entities={ptEntities} labels={ptLabels} component="pressure_psi" colors={ptColors} yLabel="Pressure (PSI)" />
                        </div>
                    </div>
                    <div className="flex-1 flex flex-col gap-1.5 min-h-0 min-w-0">
                        <MovingAverageReadout
                            label="TC avg"
                            value={tcMa}
                            unit="°C"
                            decimals={1}
                        />
                        <div className="flex-1 bg-card rounded-lg p-2 flex flex-col min-h-0 min-w-0">
                            <DerivedTimeSeriesPlot
                                title="TC Temperatures"
                                entities={tcEntities}
                                component={tcComponent}
                                transform={tcTransform}
                                yLabel="Temperature (°C)"
                                labels={tcLabels}
                                colors={tcColors}
                                windowSeconds={60}
                            />
                        </div>
                    </div>
                    <div className="flex-1 flex flex-col gap-1.5 min-h-0 min-w-0">
                        <MovingAverageReadout
                            label="LC avg"
                            value={lcMa}
                            unit="N"
                            decimals={1}
                        />
                        <div className="flex-1 bg-card rounded-lg p-2 flex flex-col min-h-0 min-w-0">
                            <DerivedTimeSeriesPlot
                                title="LC Forces"
                                entities={lcEntities}
                                component={lcComponent}
                                transform={lcTransform}
                                yLabel="Force (N)"
                                labels={lcLabels}
                                colors={lcColors}
                                windowSeconds={60}
                            />
                        </div>
                    </div>
                </div>

                <div className="flex-shrink-0 min-h-[72px] overflow-auto">
                    <ActuatorStatePanel
                        compact
                        title="Chamber Actuators"
                        actuators={[
                            { label: 'LOX Main', entity: 'ACT.LOX_Main', color: getActuatorColor('ACT.LOX_Main') },
                            { label: 'Fuel Main', entity: 'ACT.Fuel_Main', color: getActuatorColor('ACT.Fuel_Main') },
                        ]}
                    />
                </div>
            </div>

        </main>
    );
}

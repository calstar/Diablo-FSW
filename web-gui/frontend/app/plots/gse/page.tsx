'use client'

import { useEffect, useState } from 'react';
import TimeSeriesPlot from '@/components/plots/TimeSeriesPlot';
import PressureBar from '@/components/plots/PressureBar';
import ActuatorStatePanel from '@/components/plots/ActuatorStatePanel';
import SensorReadoutStrip from '@/components/plots/SensorReadoutStrip';
import { useSensorStore, useSensorValue } from '@/lib/store';
import { getWebSocketClient } from '@/lib/websocket';
import { MessageType, SensorUpdate, StateUpdate } from '@/lib/types';
import { getEntityColor, getActuatorColor, PRESSURE_SENSORS } from '@/lib/sensor-colors';
import { useSensorConfig, filterByRole } from '@/lib/sensor-config';

function getSensorLimits(entity: string) {
  const s = PRESSURE_SENSORS.find((p) => p.entity === entity);
  return { nop: s?.nop ?? 500, meop: s?.meop ?? 700 };
}

type GSETab = 'pressurant' | 'fuel' | 'lox';

const TAB_CONFIG: Record<GSETab, {
  label: string;
  color: string;
  accentClass: string;
  activeClass: string;
  entities: string[];
  entityLabels: string[];
  yLabel: string;
  actuators: { label: string; entity: string }[];
  barLabel: string;
  barEntity: string;
}> = {
  pressurant: {
    label: 'Pressurant Fill',
    color: '#16A085',
    accentClass: 'bg-teal-500',
    activeClass: 'bg-teal-500 text-black',
    entities: ['PT_Cal.GSE_Mid', 'PT_Cal.GN2_High'],
    entityLabels: ['GSE Mid (HP Source)', 'GN2 High (Vehicle)'],
    yLabel: 'Pressure (PSI)',
    actuators: [
      { label: 'GN2 Vent',        entity: 'ACT.GN2_Vent' },
      { label: 'Med Press Ctrl',  entity: 'ACT.GSE_Med_Press_Control' },
      { label: 'High Press Ctrl', entity: 'ACT.GSE_High_Press_Control' },
      { label: 'High Press Vent', entity: 'ACT.GSE_High_Press_Vent' },
    ],
    barLabel: 'Mid',
    barEntity: 'PT_Cal.GSE_Mid',
  },
  fuel: {
    label: 'Fuel Fill',
    color: '#F39C12',
    accentClass: 'bg-orange-500',
    activeClass: 'bg-orange-500 text-black',
    entities: ['PT_Cal.GSE_Low'],
    entityLabels: ['GSE Low (Fuel Fill)'],
    yLabel: 'Pressure (PSI)',
    actuators: [
      { label: 'GSE Low Vent',    entity: 'ACT.GSE_Low_Vent' },
      { label: 'Fuel Fill Vent',  entity: 'ACT.Fuel_Fill_Vent' },
      { label: 'Fuel Fill Press', entity: 'ACT.Fuel_Fill_Press' },
    ],
    barLabel: 'Low',
    barEntity: 'PT_Cal.GSE_Low',
  },
  lox: {
    label: 'LOX Fill',
    color: '#8E44AD',
    accentClass: 'bg-purple-500',
    activeClass: 'bg-purple-500 text-black',
    entities: ['PT_Cal.GSE_High'],
    entityLabels: ['GSE High (LOX Pressurant)'],
    yLabel: 'Pressure (PSI)',
    actuators: [
      { label: 'GSE High Vent',   entity: 'ACT.GSE_High_Press_Vent' },
      { label: 'High Press Ctrl', entity: 'ACT.GSE_High_Press_Control' },
      { label: 'LOX Fill Vent',   entity: 'ACT.GSE_LOX_Fill_Vent' },
      { label: 'LOX Fill',        entity: 'ACT.LOX_Fill' },
      { label: 'LOX Dump',        entity: 'ACT.LOX_Dump' },
    ],
    barLabel: 'High',
    barEntity: 'PT_Cal.GSE_High',
  },
};

export default function GSEGraphsPage() {
  const updateSensor = useSensorStore((s) => s.updateSensor);
  const updateState = useSensorStore((s) => s.updateState);
  const ws = getWebSocketClient();
  const allSensors = useSensorConfig();

  const [activeTab, setActiveTab] = useState<GSETab>('pressurant');
  const tab = TAB_CONFIG[activeTab];

  const gseSensors = filterByRole(allSensors, 'GSE');

  const pressurantLimits = getSensorLimits('PT_Cal.GSE_Mid');
  const fuelLimits       = getSensorLimits('PT_Cal.GSE_Low');
  const loxLimits        = getSensorLimits('PT_Cal.GSE_High');

  const midVal  = useSensorValue('PT_Cal.GSE_Mid',  'pressure_psi');
  const lowVal  = useSensorValue('PT_Cal.GSE_Low',  'pressure_psi');
  const highVal = useSensorValue('PT_Cal.GSE_High', 'pressure_psi');

  // All three GSE pressures for graph and readout (tab only switches actuators)
  const gsePressureEntities = ['PT_Cal.GSE_Mid', 'PT_Cal.GSE_Low', 'PT_Cal.GSE_High'] as const;
  const gsePressureLabels = ['GSE Mid', 'GSE Low', 'GSE High'];

  const readoutSensors = gsePressureEntities.map((entity, i) => ({
    label: gsePressureLabels[i],
    entity,
    component: 'pressure_psi' as const,
    color: getEntityColor(entity),
  }));

  useEffect(() => {
    ws.connect();
    const unsub1 = ws.on(MessageType.SENSOR_UPDATE, (p: unknown) => updateSensor(p as SensorUpdate));
    const unsub2 = ws.on(MessageType.STATE_UPDATE, (p: unknown) => updateState(p as StateUpdate));
    return () => { unsub1(); unsub2(); };
  }, [ws, updateSensor, updateState]);

  return (
    <main className="h-full bg-background text-text flex flex-col overflow-hidden p-2 gap-1">

      {/* Header + tab row */}
      <div className="flex items-center justify-between flex-shrink-0">
        <div className="flex items-center gap-2">
          <div className={`w-0.5 h-4 rounded-full ${tab.accentClass}`} />
          <h1 className="text-sm font-bold tracking-wider" style={{ color: tab.color }}>GSE — {tab.label.toUpperCase()}</h1>
        </div>
        <div className="flex gap-1 bg-gray-900 rounded-lg p-0.5">
          {(Object.keys(TAB_CONFIG) as GSETab[]).map((key) => (
            <button
              key={key}
              onClick={() => setActiveTab(key)}
              className={`px-2 py-1 text-xs font-bold rounded transition-colors ${
                activeTab === key
                  ? TAB_CONFIG[key].activeClass
                  : 'text-gray-400 hover:text-white hover:bg-gray-800'
              }`}
            >
              {TAB_CONFIG[key].label}
            </button>
          ))}
        </div>
      </div>

      {/* Live readout strip – compact */}
      <div className="flex-shrink-0">
        <SensorReadoutStrip variant="compact" sensors={readoutSensors} />
      </div>

      {/* Body: chart + actuators + pressure bar */}
      <div className="flex-1 min-h-0 flex flex-row gap-2">

        <div className="flex-1 flex flex-col gap-1.5 min-h-0 min-w-0">
          <div className="flex-[5] min-h-0 bg-card rounded-lg p-2 flex flex-col min-w-0">
            <TimeSeriesPlot
              title="GSE Pressures"
              entities={[...gsePressureEntities]}
              labels={[...gsePressureLabels]}
              component="pressure_psi"
              colors={gsePressureEntities.map((e) => getEntityColor(e))}
              yLabel="Pressure (PSI)"
            />
          </div>

          <div className="flex-shrink-0 min-h-[140px] overflow-auto">
            <ActuatorStatePanel
              compact
              title={`${tab.label} Actuators`}
              actuators={tab.actuators.map((a) => ({ ...a, color: getActuatorColor(a.entity) }))}
            />
          </div>
        </div>

        {/* Pressure bars: all three GSE Mid, Low, High */}
        <div className="w-40 bg-card rounded-lg p-3 flex flex-col gap-2 flex-shrink-0 overflow-visible">
          <div className="text-xs font-bold uppercase tracking-widest text-gray-400 text-center flex-shrink-0">
            Pressures
          </div>
          <div className="flex flex-row flex-1 gap-2 min-h-0 overflow-visible w-full pr-6">
            <div className="flex-1 min-h-0 min-w-0 max-w-full overflow-visible">
              <PressureBar label="Mid" value={midVal} nop={pressurantLimits.nop} meop={pressurantLimits.meop} color={getEntityColor('PT_Cal.GSE_Mid')} showLabels={false} />
            </div>
            <div className="flex-1 min-h-0 min-w-0 max-w-full overflow-visible">
              <PressureBar label="Low" value={lowVal} nop={fuelLimits.nop} meop={fuelLimits.meop} color={getEntityColor('PT_Cal.GSE_Low')} showLabels={false} />
            </div>
            <div className="flex-1 min-h-0 min-w-0 max-w-full overflow-visible">
              <PressureBar label="High" value={highVal} nop={loxLimits.nop} meop={loxLimits.meop} color={getEntityColor('PT_Cal.GSE_High')} showLabels={false} />
            </div>
          </div>
        </div>

      </div>
    </main>
  );
}

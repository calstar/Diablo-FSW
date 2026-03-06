'use client'

import { useMemo } from 'react';
import { useSensorStore } from '@/lib/store';
import { getDataCache } from '@/lib/data-cache';
import { getStartupTime } from '@/lib/startup-time';

const HISTORY_WINDOW_SEC = 60;
const DEFAULT_MA_WINDOW_SEC = 5;

/**
 * Compute moving average of the mean across entities over the last maWindowSeconds.
 * Uses aligned history from the data cache; re-runs when sensorData updates.
 */
export function useMovingAverage(
  entities: string[],
  component: string,
  maWindowSeconds: number = DEFAULT_MA_WINDOW_SEC
): number | null {
  const sensorData = useSensorStore((s) => s.sensorData);

  return useMemo(() => {
    if (entities.length === 0) return null;

    const cache = getDataCache();
    const componentMap = entities.map(() => component);
    const aligned = cache.getAlignedHistory(entities, componentMap, HISTORY_WINDOW_SEC);
    if (!aligned || aligned.time.length === 0) return null;

    const now = (Date.now() - getStartupTime()) / 1000;
    const cutoff = now - maWindowSeconds;

    const { time, values } = aligned;
    const n = time.length;
    const numEntities = values.length;

    // At each time index: average across entities (skip NaN)
    const avgAtTick: number[] = [];
    for (let j = 0; j < n; j++) {
      let sum = 0;
      let count = 0;
      for (let e = 0; e < numEntities; e++) {
        const v = values[e][j];
        if (isFinite(v)) {
          sum += v;
          count++;
        }
      }
      avgAtTick.push(count > 0 ? sum / count : NaN);
    }

    // Trailing average over points within maWindowSeconds
    let sum = 0;
    let count = 0;
    for (let j = n - 1; j >= 0; j--) {
      if (time[j] < cutoff) break;
      if (isFinite(avgAtTick[j])) {
        sum += avgAtTick[j];
        count++;
      }
    }
    if (count === 0) return null;
    return sum / count;
  }, [entities.join(','), component, maWindowSeconds, sensorData]);
}

export type DerivedTransformFn = (rawValue: number) => number | null;

/**
 * Moving average of transformed values: get raw history, apply transform at each point,
 * then average across entities per tick, then trailing average over maWindowSeconds.
 */
export function useMovingAverageDerived(
  entities: string[],
  component: string,
  transform: DerivedTransformFn,
  maWindowSeconds: number = DEFAULT_MA_WINDOW_SEC
): number | null {
  const sensorData = useSensorStore((s) => s.sensorData);

  return useMemo(() => {
    if (entities.length === 0) return null;

    const cache = getDataCache();
    const componentMap = entities.map(() => component);
    const aligned = cache.getAlignedHistory(entities, componentMap, HISTORY_WINDOW_SEC);
    if (!aligned || aligned.time.length === 0) return null;

    const now = (Date.now() - getStartupTime()) / 1000;
    const cutoff = now - maWindowSeconds;

    const { time, values } = aligned;
    const n = time.length;
    const numEntities = values.length;

    const avgAtTick: number[] = [];
    for (let j = 0; j < n; j++) {
      let sum = 0;
      let count = 0;
      for (let e = 0; e < numEntities; e++) {
        const raw = values[e][j];
        if (!isFinite(raw)) continue;
        const converted = transform(raw);
        if (converted !== null && isFinite(converted)) {
          sum += converted;
          count++;
        }
      }
      avgAtTick.push(count > 0 ? sum / count : NaN);
    }

    let sum = 0;
    let count = 0;
    for (let j = n - 1; j >= 0; j--) {
      if (time[j] < cutoff) break;
      if (isFinite(avgAtTick[j])) {
        sum += avgAtTick[j];
        count++;
      }
    }
    if (count === 0) return null;
    return sum / count;
  }, [entities.join(','), component, maWindowSeconds, sensorData, transform]);
}

interface MovingAverageReadoutProps {
  label: string;
  value: number | null;
  unit: string;
  decimals?: number;
  className?: string;
}

export function MovingAverageReadout({
  label,
  value,
  unit,
  decimals = 1,
  className = '',
}: MovingAverageReadoutProps) {
  const display =
    value !== null && isFinite(value)
      ? value.toLocaleString('en-US', {
          minimumFractionDigits: decimals,
          maximumFractionDigits: decimals,
        })
      : '---';

  return (
    <div
      className={`flex items-center justify-between gap-2 rounded-lg border border-white/10 bg-white/[0.03] px-3 py-1.5 ${className}`}
    >
      <span className="text-[10px] font-bold uppercase tracking-widest text-gray-400 truncate">
        {label}
      </span>
      <span className="text-sm font-black font-mono tabular-nums text-white">
        {display}
      </span>
      <span className="text-[9px] font-semibold uppercase tracking-wider text-gray-500">
        {unit}
      </span>
    </div>
  );
}

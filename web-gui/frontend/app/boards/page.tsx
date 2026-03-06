'use client'

import { useCallback, useEffect, useMemo } from 'react';
import { useSensorStore } from '@/lib/store';
import { getWebSocketClient } from '@/lib/websocket';
import { MessageType, BoardStatusPayload, BoardStatus, engineStateCodeToLabel, CommandPayload } from '@/lib/types';

function formatConfigSentAt(ms: number | undefined): string {
  if (ms == null) return '';
  const d = new Date(ms);
  return d.toLocaleTimeString(undefined, { hour12: false, hour: '2-digit', minute: '2-digit', second: '2-digit' });
}

// Column order: one column per board type (only enabled boards from config appear)
const BOARD_TYPE_ORDER = ['PT', 'LC', 'TC', 'RTD', 'ACTUATOR'] as const;
const TYPE_ACCENT: Record<string, string> = {
  PT: 'border-l-emerald-500/70 bg-emerald-950/10',
  LC: 'border-l-blue-500/70 bg-blue-950/10',
  TC: 'border-l-violet-500/70 bg-violet-950/10',
  RTD: 'border-l-amber-500/70 bg-amber-950/10',
  ACTUATOR: 'border-l-rose-500/70 bg-rose-950/10',
};
const CARD_ACCENTS = Object.values(TYPE_ACCENT);

function BoardCard({ b, accent }: { b: BoardStatus; accent: string }) {
  const unexpected = !b.expected;
  const freq =
    b.frequencyHz != null && isFinite(b.frequencyHz)
      ? `${b.frequencyHz.toFixed(1)} Hz`
      : '---';
  let boardStateLabel = 'Unknown';
  if (b.boardState === 1) boardStateLabel = 'Setup';
  else if (b.boardState === 2) boardStateLabel = 'Active';
  else if (b.boardState === 3) boardStateLabel = 'Abort';
  else if (b.boardState === 4) boardStateLabel = 'Abort done';
  const engineLabel = engineStateCodeToLabel(b.engineState);
  const title = `${b.type || 'UNKNOWN'} #${b.id % 10}`;

  return (
    <div
      className={`rounded-lg border-l-4 p-2.5 border border-gray-700 transition-colors flex flex-col w-[280px] h-[200px] flex-shrink-0
        ${unexpected ? 'bg-yellow-950/40 border-yellow-600 border-l-yellow-500' : `bg-card hover:border-gray-600 ${accent}`}`}
    >
      <div className="flex items-center justify-between gap-1 mb-1.5">
        <h3 className="text-xs font-bold tracking-wider text-text uppercase truncate min-w-0">
          {title}
        </h3>
        {unexpected && (
          <span className="text-[10px] font-bold text-yellow-400 uppercase tracking-wider flex-shrink-0">
            UNEXP
          </span>
        )}
      </div>
      <div className="flex flex-wrap gap-x-3 gap-y-1 mb-1 text-xs">
        <div className="flex items-center gap-1.5 min-w-0">
          <div className="text-text-muted text-[10px] uppercase tracking-wider flex-shrink-0">Status</div>
          <div className={`flex items-center gap-1 ${!(b.operational ?? b.connected) ? 'text-red-400' : 'text-green-400'}`}>
            <div
              className={`w-2 h-2 rounded-full flex-shrink-0 ${!(b.operational ?? b.connected) ? 'bg-red-500' : 'bg-green-500'}`}
            />
            <span className="font-mono font-bold truncate">
              {(b.operational ?? b.connected) ? 'OK' : 'DOWN'}
            </span>
          </div>
        </div>
        <div className="flex items-center gap-1.5 min-w-0">
          <div className="text-text-muted text-[10px] uppercase tracking-wider flex-shrink-0">State</div>
          <div className="flex items-center gap-1 text-text">
            <div
              className={`w-2 h-2 rounded-full flex-shrink-0 ${boardStateLabel === 'Active' ? 'bg-green-500' :
                  boardStateLabel === 'Setup' ? 'bg-blue-500' :
                    boardStateLabel === 'Abort' || boardStateLabel === 'Abort done' ? 'bg-red-500' : 'bg-gray-500'
                }`}
            />
            <span className="font-mono font-bold truncate">
              {boardStateLabel.toUpperCase()}
            </span>
          </div>
        </div>
      </div>
      <div className="text-[10px] text-text-muted font-mono mb-0.5">
        Engine: {engineLabel}
      </div>
      {b.configured !== undefined && (
        <div className="flex items-center gap-1.5 mb-0.5 flex-wrap">
          <span
            className={`text-[10px] px-1.5 py-0.5 rounded font-semibold uppercase tracking-wide font-mono ${b.configured ? 'bg-emerald-900/60 text-emerald-200' : 'bg-gray-800 text-gray-500'
              }`}
          >
            {b.configured ? 'Config sent' : 'Unconfigured'}
          </span>
          {b.configured && b.configLastSentAt != null && (
            <span className="text-[10px] text-emerald-400/90 font-mono">
              {formatConfigSentAt(b.configLastSentAt)}
            </span>
          )}
        </div>
      )}
      {b.configError && (
        <div className="text-[10px] text-red-400 font-mono mb-0.5 truncate" title={b.configError}>
          Config err: {b.configError}
        </div>
      )}
      <div className="text-[10px] text-text-muted font-mono">
        HB: {freq}
      </div>
      <div className="text-[10px] text-gray-500 font-mono mt-auto pt-1 truncate" title={b.ip}>
        ID {b.id} · {b.ip}
      </div>
    </div>
  );
}

export default function BoardsPage() {
  const updateBoards = useSensorStore((s) => s.updateBoards);
  const boardsMap = useSensorStore((s) => s.boards as Record<number, BoardStatus>);
  const ws = getWebSocketClient();

  const { byType, unexpected, boards } = useMemo(() => {
    const map = boardsMap ?? {};
    const list = Object.values(map);
    const expected = list.filter((b) => b.expected);
    const unexpectedList = list.filter((b) => !b.expected);
    const byType: Record<string, BoardStatus[]> = {};
    for (const type of BOARD_TYPE_ORDER) {
      byType[type] = [];
    }
    for (const b of expected) {
      const type = b.type || 'UNKNOWN';
      if (!byType[type]) byType[type] = [];
      byType[type].push(b);
    }
    for (const type of Object.keys(byType)) {
      byType[type].sort((a, b) => {
        const an = a.boardNumber ?? Number.MAX_SAFE_INTEGER;
        const bn = b.boardNumber ?? Number.MAX_SAFE_INTEGER;
        if (an !== bn) return an - bn;
        return a.id - b.id;
      });
    }
    const boards = [...expected, ...unexpectedList].sort((a, b) => a.id - b.id);
    return { byType, unexpected: unexpectedList, boards };
  }, [boardsMap]);

  useEffect(() => {
    ws.connect();
    const unsub = ws.on(MessageType.BOARD_STATUS_UPDATE, (p: unknown) => {
      const payload = p as BoardStatusPayload;
      if (payload?.boards) updateBoards(payload.boards as BoardStatus[]);
    });
    return () => unsub();
  }, [ws, updateBoards]);

  const handleResendConfig = useCallback(() => {
    getWebSocketClient().send({
      type: MessageType.RESEND_CONFIG,
      timestamp: Date.now(),
      payload: {},
    });
  }, []);

  const handleClearAbort = useCallback(() => {
    const cmd: CommandPayload = { commandType: 'clear_abort', data: {} };
    getWebSocketClient().sendCommand(cmd);
  }, []);

  return (
    <main className="h-full min-h-0 bg-background text-text flex flex-col overflow-hidden p-2">
      <div className="flex-shrink-0 flex flex-wrap items-center justify-between gap-2 mb-2">
        <div className="min-w-0">
          <h1 className="text-lg font-bold text-text tracking-tight">Boards / Heartbeats</h1>
          <p className="text-xs text-text-muted max-w-xl truncate">
            Enabled boards from config, grouped by type. Unexpected (discovered) boards appear in their own column.
          </p>
        </div>
        <div className="flex flex-wrap items-center gap-3">
          <button
            type="button"
            onClick={handleClearAbort}
            className="min-h-[48px] px-6 py-3 text-base font-bold rounded-lg border border-red-500/70 bg-red-950/40 text-red-200 hover:bg-red-900/50 transition-colors"
          >
            Clear ABORT
          </button>
          <button
            type="button"
            onClick={handleResendConfig}
            className="min-h-[48px] px-8 py-3 text-lg font-bold rounded-lg bg-primary text-primary-foreground hover:opacity-90 transition-opacity shadow-lg"
          >
            Resend config
          </button>
        </div>
      </div>

      <div className="flex-1 min-h-0 overflow-auto">
        {boards.length === 0 ? (
          <div className="rounded-xl border border-gray-700 bg-card p-12 text-center text-text-muted text-lg">
            No boards configured or discovered yet. Ensure the backend is running and broadcasting SERVER_HEARTBEAT.
          </div>
        ) : (
          <div className="grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-3 xl:grid-cols-4 gap-6">
            {boards.map((b, index) => {
              const unexpected = !b.expected;
              const accent = CARD_ACCENTS[index % CARD_ACCENTS.length];
              const freq =
                b.frequencyHz != null && isFinite(b.frequencyHz)
                  ? `${b.frequencyHz.toFixed(1)} Hz`
                  : '---';
              let boardStateLabel = 'Unknown';
              if (b.boardState === 1) boardStateLabel = 'Setup';
              else if (b.boardState === 2) boardStateLabel = 'Active';
              else if (b.boardState === 3) boardStateLabel = 'Abort';
              else if (b.boardState === 4) boardStateLabel = 'Abort done';
              else if (b.boardState === 5) boardStateLabel = 'Conn Loss';
              else if (b.boardState === 6) boardStateLabel = 'No Conn Abort';
              else if (b.boardState === 7) boardStateLabel = 'No Conn Abort (F)';
              else if (b.boardState === 8) boardStateLabel = 'PT Abort';
              else if (b.boardState === 9) boardStateLabel = 'No PT Abort';
              else if (b.boardState === 10) boardStateLabel = 'Abort Finished';
              const engineLabel = engineStateCodeToLabel(b.engineState);
              const nameParts = [];
              if (b.type) nameParts.push(b.type);
              if (b.boardNumber != null) nameParts.push(`Board ${b.boardNumber}`);
              const title = nameParts.join(' · ') || `ID ${b.id}`;
              return (
                <div
                  key={b.id}
                  className={`rounded-xl border-l-4 p-6 border border-gray-700 transition-colors min-h-[200px] flex flex-col
                  ${unexpected ? 'bg-yellow-950/40 border-yellow-600 border-l-yellow-500' : `bg-card hover:border-gray-600 ${accent}`}`}
                >
                  <div className="flex items-center justify-between mb-4">
                    <h3 className="text-xl font-bold tracking-wider text-text uppercase truncate pr-3">
                      {title}
                    </h3>
                    {unexpected && (
                      <span className="text-sm font-bold text-yellow-400 uppercase tracking-wider flex-shrink-0">
                        UNEXPECTED
                      </span>
                    )}
                  </div>
                  <div className="flex flex-wrap gap-4 mb-3 text-lg">
                    <div className="flex-1 min-w-0">
                      <div className="text-text-muted mb-1.5 text-sm uppercase tracking-wider">Status</div>
                      <div className={`flex items-center gap-2.5 ${!b.connected ? 'text-red-400' : 'text-green-400'}`}>
                        <div
                          className={`w-3.5 h-3.5 rounded-full flex-shrink-0 ${!b.connected ? 'bg-red-500' : 'bg-green-500'}`}
                        />
                        <span className="font-mono font-bold text-lg truncate">
                          {b.connected ? 'CONNECTED' : 'DISCONNECTED'}
                        </span>
                      </div>
                    </div>
                    <div className="flex-1 min-w-0">
                      <div className="text-text-muted mb-1.5 text-sm uppercase tracking-wider">State</div>
                      <div className="flex items-center gap-2.5 text-text">
                        <div
                          className={`w-3.5 h-3.5 rounded-full flex-shrink-0 ${boardStateLabel === 'Active' ? 'bg-green-500' :
                            boardStateLabel === 'Setup' ? 'bg-blue-500' :
                              boardStateLabel === 'Conn Loss' ? 'bg-yellow-500' :
                                boardStateLabel.includes('Abort') ? 'bg-red-500' : 'bg-gray-500'
                            }`}
                        />
                        <span className="font-mono font-bold text-lg truncate">
                          {boardStateLabel.toUpperCase()}
                        </span>
                      </div>
                    </div>
                  </div>
                  <div className="text-sm text-text-muted font-mono mb-2">
                    Engine: {engineLabel}
                  </div>
                  {b.configured !== undefined && (
                    <div className="flex items-center gap-2 mb-2">
                      <span
                        className={`text-xs px-2 py-1 rounded font-semibold uppercase tracking-wide font-mono ${b.configured ? 'bg-emerald-900/60 text-emerald-200' : 'bg-gray-800 text-gray-500'
                          }`}
                      >
                        {b.configured ? 'Config sent' : 'Unconfigured'}
                      </span>
                      {b.configured && b.configLastSentAt != null && (
                        <span className="text-xs text-emerald-400/90 font-mono">
                          at {formatConfigSentAt(b.configLastSentAt)}
                        </span>
                      )}
                    </div>
                  )}
                  {b.configError && (
                    <div className="text-xs text-red-400 font-mono mb-2" title={b.configError}>
                      Config error: {b.configError}
                    </div>
                  )}
                  <div className="text-base text-text-muted font-mono mb-2">
                    Heartbeat: {freq}
                  </div>
                  <div className="text-sm text-gray-500 font-mono mt-auto pt-3 truncate" title={b.ip}>
                    ID {b.id} · {b.ip}
                  </div>
                </div>
              );
            })}
          </div>
        )}
      </div>
    </main>
  );
}

export function LiveFlightSteps({ progress, connection }) {
  const lines = Array.isArray(progress?.lines) ? progress.lines : []
  const latest = progress?.latestLine
  const repeats = progress?.repeatCount > 1 ? ` — still the same, ${progress.repeatCount} times` : ""
  let connectionText = "Connecting to live steps..."
  if (connection === "open") connectionText = "Live steps are connected."
  if (connection === "closed") connectionText = "Live steps closed."
  if (connection === "error") connectionText = "Live steps did not connect."

  return (
    <section className="logList flightSteps">
      <h2>Live steps</h2>
      <p>{connectionText}</p>
      {latest ? <p className="currentDirection">{latest}{repeats}</p> : <p>No flight step yet. Press Fly and each step appears here while it happens.</p>}
      {progress?.running ? <p>The flight is in progress.</p> : null}
      {progress?.outcome && !progress.running ? <p>Outcome: {progress.outcome}</p> : null}
      {progress?.stopReason ? <p>Stop reason: {progress.stopReason}</p> : null}
      <ol className="positionList">
        {lines.map((line, index) => (
          <li key={`${index}-${line}`} className={index === lines.length - 1 ? "currentPosition" : ""}>
            {line}
          </li>
        ))}
      </ol>
    </section>
  )
}

/**
 * @param { Date | string | number | null } [datetime]
 */
function _getDateTimeComponents(datetime) {
  const d = datetime ? new Date(datetime) : new Date();

  return {
    y: d.getFullYear(),
    m: String(d.getMonth() + 1).padStart(2, "0"),
    d: String(d.getDate()).padStart(2, "0"),
    h: String(d.getHours()).padStart(2, "0"),
    i: String(d.getMinutes()).padStart(2, "0"),
    s: String(d.getSeconds()).padStart(2, "0")
  };
}

export function getTodayString() {
  const { y, m, d } = _getDateTimeComponents();
  return `${y}-${m}-${d}`;
}

export function getCurrentTimeString() {
  const { h, i, s } = _getDateTimeComponents();
  return `${h}:${i}:${s}`;
}

export function getCurrentDatetimeString() {
  const { y, m, d, h, i, s } = _getDateTimeComponents();
  return `${y}-${m}-${d} ${h}:${i}:${s}`;
}

export function getDatetimeString(datetime) {
  const { y, m, d, h, i, s } = _getDateTimeComponents(datetime);
  return `${y}-${m}-${d} ${h}:${i}:${s}`;
}

/**
 * @param {string} date - "YYYY-MM-DD"
 * @returns {number}
 *  -1 : past
 *   0 : today
 *   1 : future
 */
export function compareWithToday(date) {
	const today = getTodayString();

	if (today > date) return -1;
	if (date > today) return 1;
	return 0;
}

/**
 * @param {string} time - "HH:mm:ss"
 * @returns {boolean}
 */
export function isFutureTime(time) {
  return time > getCurrentTimeString();
}

/**
 * 
 * @param {string} date e.g. "YYYY-MM-DD"
 * @param {number} offset
 * @return {string} date - e.g. "YYYY-MM-DD"
 */
export function generateOffsetDate(date, offset) {
	const [y, m, d] = date.split('-').map(Number);
  const dateObj = new Date(y, m - 1, d + offset);

	const year = dateObj.getFullYear();
	const month = String(dateObj.getMonth() + 1).padStart(2, '0');
	const day = String(dateObj.getDate()).padStart(2, '0');

	return `${year}-${month}-${day}`;
}

/**
 * Calculates the elapsed time between two datetime strings and returns it in HH:mm:ss format.
 * @param {string} start - The starting datetime (e.g., "YYYY-MM-DD HH:mm:ss").
 * @param {string} end - The ending datetime (e.g., "YYYY-MM-DD HH:mm:ss").
 * @returns {string} The formatted elapsed time string (HH:mm:ss).
 */
function getElapsedString(start, end) {
  const dStart = new Date(start.replace(" ", "T"));
  const dEnd = new Date(end.replace(" ", "T"));

  const diffSec = Math.floor((dEnd - dStart) / 1000);

  if (diffSec < 0) return "00:00:00";

  const h = Math.floor(diffSec / 3600);
  const m = Math.floor((diffSec % 3600) / 60);
  const s = diffSec % 60;

  return [h, m, s].map((v) => String(v).padStart(2, "0")).join(":");
}

function startClock($dom) {
  const now = new Date();

  const year = now.getFullYear();
  const month = now.getMonth() + 1;
  const day = now.getDate();
  const hours = now.getHours();
  const minutes = now.getMinutes();
  const seconds = now.getSeconds();

  $dom.textContent = `${year}-${month}-${day} ${String(hours).padStart(2, "0")}:${String(minutes).padStart(2, "0")}`;

  const delay = 60 - seconds;

  setTimeout(() => {
    this._startClock();
  }, delay * 1000);
}

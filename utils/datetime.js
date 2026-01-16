function _getNowComponents() {
  const now = new Date();
  return {
    y: now.getFullYear(),
    m: String(now.getMonth() + 1).padStart(2, "0"),
    d: String(now.getDate()).padStart(2, "0"),
    h: String(now.getHours()).padStart(2, "0"),
    i: String(now.getMinutes()).padStart(2, "0"),
    s: String(now.getSeconds()).padStart(2, "0"),
  };
}

function getTodayString() {
  const { y, m, d } = _getNowComponents();
  return `${y}-${m}-${d}`;
}

function getCurrentTimeString() {
  const { h, i, s } = _getNowComponents();
  return `${h}:${i}:${s}`;
}

export function getCurrentDatetimeString() {
  const { y, m, d, h, i, s } = _getNowComponents();
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
  const [y, m, d] = date.split("-").map(Number);
  const dateObj = new Date(y, m - 1, d + offset);

  const year = dateObj.getFullYear();
  const month = String(dateObj.getMonth() + 1).padStart(2, "0");
  const day = String(dateObj.getDate()).padStart(2, "0");

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
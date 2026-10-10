"""Check the supplied native files against Inventor's saved export manifest."""
import csv
import hashlib
import json
from pathlib import Path

PKG = Path(__file__).resolve().parents[1]
SOURCE = PKG.parent / 'GPT6'

def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def main():
    rows = list(csv.DictReader((SOURCE/'00_Documentacion/INVENTARIO_ARCHIVOS_ACTIVOS.csv').open(encoding='utf-8-sig')))
    result = []
    for row in rows:
        relative = row['archivo'].replace('\\', '/').split('/GPT6/', 1)[1]
        path = SOURCE / relative
        actual = sha256(path) if path.exists() else None
        result.append(dict(path=relative, expected_sha256=row['sha256'], actual_sha256=actual, matches=actual == row['sha256']))
    report = {'checked':len(result), 'matching':sum(r['matches'] for r in result), 'files':result}
    (PKG/'docs').mkdir(exist_ok=True)
    (PKG/'docs/source_audit.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k!='files'}))
    print('Mismatches:', [r['path'] for r in result if not r['matches']])
    if not all(r['matches'] for r in result):
        raise RuntimeError('Native files differ from the export manifest')

if __name__ == '__main__':
    main()

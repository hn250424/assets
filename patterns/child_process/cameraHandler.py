import sys
import json

camera_connected = False
capture_count = 0  # 캡처 횟수 카운트

def main():
    global camera_connected, capture_count
    try:
        while True:
            # 명령어 입력 받기
            command_line = sys.stdin.readline().strip()
            parts = command_line.split()  # 공백을 기준으로 명령어와 파라미터 분리
            
            response = {
                'result': 1,
                'data': ''
            }

            if len(parts) == 0:
                continue  # 빈 명령어 리턴

            command = parts[0]  # 명령어
            params = parts[1:]  # 명령어에 따른 파라미터

            if command == 'connect':
                if len(params) == 1:
                    serialNo = params[0]
                    camera_connected = True
                    
                    response['data'] = f'{serialNo}'
                    print(json.dumps(response))
                    sys.stdout.flush()
                else:
                    response['result'] = -1
                    response['data'] = f'{serialNo}'
                    print(json.dumps(response))
                    sys.stdout.flush()

            elif command == 'capture':
                if camera_connected:
                    if len(params) == 2:
                        try:
                            filename = params[0]  # 첫 번째 파라미터는 파일명
                            savePath = params[1]  # 두 번째 파라미터는 저장 경로
                            capture_count += 1
                            
                            response['data'] = {
                                'captureCount': f'{capture_count}',
                                'filename': f'{filename}',
                                'savePath': f'{savePath}'
                            }
                            print(json.dumps(response))
                            sys.stdout.flush()
                        except ValueError:
                            response['result'] = -1
                            response['data'] = {
                                'captureCount': f'{capture_count}',
                                'filename': f'{filename}',
                                'savePath': f'{savePath}'
                            }
                            print(json.dumps(response))
                            sys.stdout.flush()
                    else:
                        response['result'] = -1
                        response['data'] = 'Invalid parameters for capture. Expected two parameters: <filename> <savePath>'
                        print(json.dumps(response))
                        sys.stdout.flush()
                else:
                    response['result'] = -1
                    response['data'] = 'Camera not connected!'
                    print(json.dumps(response))
                    sys.stdout.flush()

            elif command == 'exit':
                response['data'] = 'Exiting...'
                print(json.dumps(response))
                sys.stdout.flush()
                break

            else:
                response['result'] = -1
                response['data'] = f'Unknown command: {command}'
                print(json.dumps(response))
                sys.stdout.flush()

    finally:
        # print('Camera script stopped')
        # sys.stdout.flush()
        pass

if __name__ == "__main__":
    # print('Camera script start')
    # sys.stdout.flush()
    main()

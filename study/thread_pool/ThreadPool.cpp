#include "ThreadPool.h"

#include <iostream>

ThreadPool::ThreadPool(size_t totalThreadSize)
	: totalThreadSize(totalThreadSize)
	, is_stop(false)
{
	// 벡터 초기화 (용량 할당 및 동작 메서드 시작). 
	threads.reserve(totalThreadSize);
	for(size_t i = 0; i < totalThreadSize; i++){
		threads.emplace_back( [this] () { this -> workJob(); } );
	}
}	// constructor

ThreadPool::~ThreadPool() {
	is_stop = true;
	cv.notify_all();
	
	// thread 객체는 복사가 불가능하기 때문에 참조자 사용. 
	for(auto& t : threads){
		t.join();
	}
}
	
void ThreadPool::workJob(){
	while(true) {
		// 먼저 뮤텍스를 선점하고, cv 체크.
		// 프로그램의 모든 스레드는 1개의 cv 대기와 나머지 뮤텍스 선점을 기다리는 스레드로 나뉘게 된다. 
		// cv notify_one()으로  깨어날 수 있는 스레드는 한 개의 대기 중인 스레드가 유일. 
		std::unique_lock<std::mutex> lock(m);
		
		// 참이 아니면 대기 상태로 진입.
		// cv.notify_one()을 통해 깨우면 다시 지정한 조건을 체크. 
		cv.wait(lock, [this] () { return !this -> jobs.empty() || is_stop; });
		
		if(is_stop && this -> jobs.empty()) return;

		std::function<void()> job = std::move(jobs.front());	
		jobs.pop();
		
		lock.unlock();
		
		job();
	}
}	// workJob()

template<class F, class... Args>
auto ThreadPool::pushJob(F&& f, Args&&... args) -> std::future<typename std::result_of<F(Args...)>::type> {
	if(is_stop) throw std::runtime_error("작업 중지");
	
	// using - 타입에 대한 별칭. 
	using return_type = typename std::result_of<F(Args...)>::type;
	
	// bind - 함수 객체 생성.
	// packaged_task - 함수를 실행하여 결괏값을 저장하는 타입. 
	// packaged_task - 리턴 값을 보관하는 타입. 
	// packaged_task<int(int)> - int를 리턴하고 인자로 int를 받는 함수. 
	// make_shared - 블록이 끝나면 지역변수 job은 사라짐.
	// 후에 jobs 객체가 job을 실행할 때 삭제된 상태. 
	// make_shared를 써서 블록이 끝나도 shared-ptr에 의해 관리되도록 함. 
	// 블록이 끝나면 job이 가리키는 메모리는 shared_ptr에 의해 관리되며, jobs 큐에 작업이 추가된 이후에도 접근 가능. 
	// forward - 완벽한 전달. 
	auto job = std::make_shared<std::packaged_task<return_type()>>(std::bind(std::forward<F>(f), std::forward<Args>(args)...));
	
	// 작업의 결과를 받기 위해 future 객체 준비.
	// 후에 job()이 실행될 때 결과를 저장. 
    std::future<return_type> job_result_future = job -> get_future();
    
	{
		std::lock_guard<std::mutex> lock(m);
		jobs.push([job]() { (*job)(); });
	}
	
	// cv notify_one()이 동작하면 스케줄러에게 신호가 가고 스케줄러가 임의로 대기 중인 스레드 하나를 깨운다. 
	cv.notify_one();
	
	return job_result_future;
}	// pushJob()

void work_1(int i) {
	printf("%d work_1... \n", i);
	std::this_thread::sleep_for(std::chrono::seconds(1));
}

int work_2(int i) {
	printf("%d work_2... \n", i);
	std::this_thread::sleep_for(std::chrono::seconds(1));
	return i;
}

std::string work_3(int i) {
	printf("%d work_3... \n", i);
	std::this_thread::sleep_for(std::chrono::seconds(1));
	return std::to_string(i);
}

int main(){
	std::cout << "main start" << std::endl;
	
	ThreadPool pool(3);
	
	std::vector<std::future<void>> void_futures;
    std::vector<std::future<int>> int_futures;
    std::vector<std::future<std::string>> string_futures;
    
    // 작업을 풀에 추가
    for (int i = 0; i < 10; i++) {
        if (i % 3 == 0) {
            void_futures.push_back(pool.pushJob([i]() { work_1(i); }));
        } else if (i % 3 == 1) {
            int_futures.push_back(pool.pushJob([i]() { return work_2(i); }));
        } else if (i % 3 == 2) {
            string_futures.push_back(pool.pushJob([i]() { return work_3(i); }));
        }
    }
	
    // 결과를 출력
    for (auto& future : void_futures) {
        future.get(); // 작업이 완료될 때까지 기다림
    }
    for (auto& future : int_futures) {
        std::cout << "Returned value from work_2: " << future.get() << std::endl;
    }
    for (auto& future : string_futures) {
        std::cout << "Returned value from work_3: " << future.get() << std::endl;
    }
	
	return 0;
}

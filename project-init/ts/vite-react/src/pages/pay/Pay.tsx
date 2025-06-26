import { Link } from 'react-router-dom';

const Pay = () => {
    return (
        <>
            <h1>Pay</h1>
            <Link to='/'>
                <button>여기는 페이, 메인 고</button>
            </Link>
        </>
    );
};

export default Pay;
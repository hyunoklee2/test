package com.example.hyunok.googledots;

import android.content.Intent;
import android.os.Bundle;
import android.os.Handler;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.view.View;
import android.view.animation.TranslateAnimation;
import android.widget.Button;
import android.widget.RelativeLayout;

public class MainActivity extends AppCompatActivity {

    private final String TAG = "test" ;
    private TranslateAnimation anim;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);


        final RelativeLayout iv = (RelativeLayout)findViewById(R.id.windows_relative);
        final com.example.mylibrary.GoogleDotThinking GoogleDotThinking =
                (com.example.mylibrary.GoogleDotThinking)findViewById(R.id.GoogleDotThinking);

        Button b = (Button)findViewById(R.id.button2);
        //GoogleDotThinking.setVisibility(View.INVISIBLE);

        b.setOnClickListener(new View.OnClickListener() {
            //@TargetApi(Build.VERSION_CODES.JELLY_BEAN)
            @Override
            public void onClick(View v) {

                new Handler().postDelayed(new Runnable()
                {
                    @Override
                    public void run()
                    {
                        //GoogleDotThinking.stop();
                        //GoogleDotThinking.setVisibility(View.INVISIBLE);
                        //finish();
                    }
                }, 500);// 0.5초 정도 딜레이를 준 후 시작
            }
        });
    }

    @Override
    protected void onDestroy() {
        Log.d(TAG,"onDestroy");
        //Debug.stopMethodTracing();
        android.os.Process.killProcess(android.os.Process.myPid());
        super.onDestroy();
        //앱의 전반적으로 돌아가던 스레드를 종료할 때 이 Debug.stopMethodTracing();
    }
}
